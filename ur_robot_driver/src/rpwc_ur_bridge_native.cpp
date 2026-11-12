#include <ur_robot_driver/rpwc_bridge_native.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iterator>
#include <sstream>

namespace
{
enum class StartupPhase { STARTING, RUNNING, FAILED };
std::mutex startup_state_mutex;
StartupPhase startup_phase = StartupPhase::STARTING;
std::string startup_rtde_failure;
bool local_control_detected_at_startup = false;

std::string pending_rtde_failure()
{
    std::lock_guard<std::mutex> lock(startup_state_mutex);
    return startup_rtde_failure;
}

void report_rtde_fatal(const std::string& reason)
{
    bool shutdown_running_bridge = false;
    {
        std::lock_guard<std::mutex> lock(startup_state_mutex);
        if (startup_phase == StartupPhase::STARTING)
        {
            startup_rtde_failure = reason;
            startup_phase = StartupPhase::FAILED;
        }
        else if (startup_phase == StartupPhase::RUNNING)
            shutdown_running_bridge = true;
    }
    if (shutdown_running_bridge)
        ros::requestShutdown();
}

template <class F>
void cleanup_step(const char* name, F&& operation)
{
    try { operation(); }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM("[Shutdown]: " << name << " failed: " << e.what());
    }
    catch (...)
    {
        ROS_ERROR_STREAM("[Shutdown]: " << name << " failed with an unknown exception");
    }
}

std::string format_tcp(const BridgeRtdeSample::Vector6& tcp)
{
    std::ostringstream out;
    out << std::setprecision(17) << '[';
    for (std::size_t i = 0; i < tcp.size(); ++i)
    {
        if (i) out << ',';
        out << tcp[i];
    }
    out << ']';
    return out.str();
}

std::string format_tcp_state(const std::string& reason)
{
    std::ostringstream out;
    out << reason;
    if (tcp_verifier_)
    {
        out << "; expected=" << format_tcp(tcp_verifier_->expected())
            << "; epoch=" << tcp_verifier_->epoch()
            << "; sequence=" << tcp_verifier_->sequence();
        BridgeRtdeSample sample;
        if (tcp_verifier_->snapshot(sample))
            out << "; latest=" << format_tcp(sample.tcp_offset)
                << "; controller_timestamp=" << std::setprecision(17)
                << sample.controller_timestamp;
    }
    return out.str();
}

template <class T>
bool finite_vector(const std::vector<T>& values)
{
    return std::all_of(values.begin(), values.end(),
        [](const T& value) { return std::isfinite(static_cast<double>(value)); });
}

bool finite_pose(const geometry_msgs::Pose& pose)
{
    const double values[] = {pose.position.x, pose.position.y, pose.position.z,
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w};
    return std::all_of(std::begin(values), std::end(values),
        [](double value) { return std::isfinite(value); });
}

class MotionGoalScope
{
public:
    MotionGoalScope() = default;
    MotionGoalScope(const MotionGoalScope&) = delete;
    MotionGoalScope& operator=(const MotionGoalScope&) = delete;
    ~MotionGoalScope() { active_motion_goal_.store(false); }
};

class ReconfigurationScope
{
public:
    ReconfigurationScope() = default;
    ReconfigurationScope(const ReconfigurationScope&) = delete;
    ReconfigurationScope& operator=(const ReconfigurationScope&) = delete;
    ~ReconfigurationScope() { reconfiguring_robot_.store(false); }
};

bool requiredRecipePresent(const std::vector<std::string>& recipe)
{
    static const char* fields[] = {"timestamp", "actual_q", "actual_qd",
        "tcp_offset", "runtime_state",
        "robot_mode", "safety_mode", "actual_digital_input_bits", "actual_digital_output_bits"};
    for (const char* field : fields)
    {
        if (std::find(recipe.begin(), recipe.end(), field) == recipe.end())
        {
            ROS_FATAL_STREAM("Required RTDE output missing after recipe negotiation: " << field);
            return false;
        }
    }
    return true;
}
}

// -----------------------------------------
//                Functions
// -----------------------------------------

bool set_init_end_status(const bool success, const std::string &msg)
{
    std::string effective_message = msg;
    {
        std::lock_guard<std::mutex> lock(startup_state_mutex);
        if (success && startup_phase == StartupPhase::FAILED)
            return false;
        if (!success && !startup_rtde_failure.empty() && startup_phase == StartupPhase::FAILED)
            effective_message = startup_rtde_failure;
        init_status_ = 1;
        init_success_ = success;
        init_msg_ = effective_message;
        startup_phase = success ? StartupPhase::RUNNING : StartupPhase::FAILED;
    }

    if (success)
        return true;

    ros::waitForShutdown();
    shutdown(effective_message);
    return true;
}

bool check_robot_mode(const urcl::RobotMode robot_mode)
{
    if (!ur_dashboard_)
    {
        throw std::invalid_argument("ur_dashboard");
    }

    std::string current_robot_mode;
    if (!ur_dashboard_->commandRobotMode(current_robot_mode))
    {
        throw urcl::UrException("Failed to get robot mode");
    }

    return current_robot_mode == urcl::robotModeString(robot_mode);
}

bool check_safety_mode(const urcl::SafetyMode safety_mode)
{
    if (!ur_dashboard_)
    {
        throw std::invalid_argument("ur_dashboard");
    }

    std::string current_safety_mode;
    if (!ur_dashboard_->commandSafetyMode(current_safety_mode))
    {
        throw urcl::UrException("Failed to get safety mode");
    }

    return current_safety_mode == urcl::safetyModeString(safety_mode);
}

void thread_handle_rtde()
{
    try
    {
    ROS_INFO("[handle_rtde]: Init");
    ros::Rate rate(freq_rtde_hz_);
    urcl::rtde_interface::DataPackage data_pkg(ur_driver_->getRTDEOutputRecipe());

    ROS_INFO("[handle_rtde]: Start");
    // Background RTDE reading makes getDataPackage() bounded (0/100 ms in
    // this URCL), allowing the reader to stop before driver teardown.
    ur_driver_->startRTDECommunication(true);

    while (bridge_threads_running_.load() && ros::ok())
    {
        if (!ur_driver_->getDataPackage(data_pkg))
        {
            if (tcp_verifier_ && tcp_verifier_->onNoPackage())
            {
                ROS_ERROR("[TCP]: RTDE stream stale; motion blocked until reconfiguration");
                if (kinematics_manager_) kinematics_manager_->invalidateRobotConfig();
                cancel_required_.store(true);
            }
            rate.sleep();
            continue;
        }

        BridgeRtdeSample sample;
        std::uint64_t in_bits = 0, out_bits = 0;
        const bool complete = data_pkg.getData<double>("timestamp", sample.controller_timestamp) &&
            data_pkg.getData<urcl::vector6d_t>("actual_q", sample.actual_q) &&
            data_pkg.getData<urcl::vector6d_t>("actual_qd", sample.actual_qd) &&
            data_pkg.getData<urcl::vector6d_t>("tcp_offset", sample.tcp_offset) &&
            data_pkg.getData<uint32_t>("runtime_state", sample.runtime_state) &&
            data_pkg.getData<int32_t>("robot_mode", sample.robot_mode) &&
            data_pkg.getData<int32_t>("safety_mode", sample.safety_mode) &&
            data_pkg.getData<std::uint64_t>("actual_digital_input_bits", in_bits) &&
            data_pkg.getData<std::uint64_t>("actual_digital_output_bits", out_bits);
        if (!complete)
        {
            ROS_FATAL("Mandatory RTDE field missing in data package; stopping bridge");
            if (tcp_verifier_) tcp_verifier_->fail(tcp_verifier_->epoch(), "Mandatory RTDE field missing");
            cancel_required_.store(true);
            report_rtde_fatal("Mandatory RTDE field missing in data package");
            break;
        }

        const auto event = tcp_verifier_->observe(sample);
        if (event == TcpVerifier::Event::DUPLICATE_SAMPLE)
            continue;
        if (event == TcpVerifier::Event::INVALID_SAMPLE)
        {
            ROS_ERROR("Invalid RTDE controller timestamp; motion blocked");
            cancel_required_.store(true);
            continue;
        }

        kinematics_manager_->updateRtdeJointData(sample.actual_q, sample.actual_qd);

        io_manager_->digital_input_bits.store(in_bits, std::memory_order_relaxed);
        io_manager_->digital_output_bits.store(out_bits, std::memory_order_relaxed);
        robot_state_manager_->updateRtdeState(sample.runtime_state, sample.robot_mode, sample.safety_mode);

        if (event != TcpVerifier::Event::NONE)
        {
            const char* name = event == TcpVerifier::Event::PROGRAM_STARTED ? "program_started" :
                               event == TcpVerifier::Event::PROGRAM_STOPPED ? "program_stopped" :
                               event == TcpVerifier::Event::TIMESTAMP_RESET ? "controller_timestamp_reset" :
                               "tcp_changed";
            ROS_WARN_STREAM("[TCP]: " << format_tcp_state(name));
            if (event != TcpVerifier::Event::PROGRAM_STARTED)
            {
                kinematics_manager_->invalidateRobotConfig();
                cancel_required_.store(true);
            }
        }

        rate.sleep();
    }

    }
    catch (const std::exception& e)
    {
        ROS_FATAL_STREAM("[handle_rtde]: RTDE reader failed: " << e.what());
        if (tcp_verifier_) tcp_verifier_->fail(tcp_verifier_->epoch(), e.what());
        if (kinematics_manager_) kinematics_manager_->invalidateRobotConfig();
        cancel_required_.store(true);
        report_rtde_fatal(std::string("RTDE reader failed: ") + e.what());
    }
    ROS_INFO("[handle_rtde]: Shutdown");
}

void thread_keep_alive()
{
    ros::Rate rate(freq_rtde_hz_);
    while (bridge_threads_running_.load() && ros::ok())
    {
        if (!robot_state_manager_ || !robot_state_manager_->isReady())
        {
            rate.sleep();
            continue;
        }

        if (send_command_mutex_.try_lock())
        {
            if (freedrive_)
                ur_driver_->writeFreedriveControlMessage(urcl::control::FreedriveControlMessage::FREEDRIVE_NOOP);
            else
                ur_driver_->writeKeepalive();

            send_command_mutex_.unlock();
        }
        rate.sleep();
    }
}

void handleRobotProgramState(bool program_running)
{
    std::lock_guard<std::mutex> callback_lock(program_callback_mutex_);
    if (!program_callback_enabled_) return;
    ur_program_connected_.store(program_running);
    if (!program_running && tcp_verifier_)
    {
        tcp_verifier_->onProgramDisconnected();
        if (kinematics_manager_) kinematics_manager_->invalidateRobotConfig();
        cancel_required_.store(true);
    }
    if (robot_state_manager_)
        robot_state_manager_->onProgramStateChanged(program_running);
}

bool motion_ready()
{
    return !reconfiguring_robot_.load() && !tool_update_failed_.load() &&
           ur_program_connected_.load() &&
           robot_state_manager_ && robot_state_manager_->isReady() &&
           tcp_verifier_ && tcp_verifier_->motionReady() &&
           kinematics_manager_ && kinematics_manager_->isRobotConfigVerified();
}

bool try_claim_motion_goal(bool& configuration_in_progress)
{
    std::lock_guard<std::mutex> lock(goal_admission_mutex_);
    configuration_in_progress = reconfiguring_robot_.load();
    if (configuration_in_progress)
        return false;
    return !active_motion_goal_.exchange(true);
}

void thread_configure_robot()
{
    while (config_worker_running_.load() && ros::ok())
    {
        // Independent freshness watch also covers a stalled RTDE background reader.
        if (tcp_verifier_->onNoPackage())
        {
            kinematics_manager_->invalidateRobotConfig();
            cancel_required_.store(true);
            ROS_ERROR("[TCP]: RTDE watchdog expired; motion blocked");
        }
        if (cancel_required_.exchange(false) && ur_instruction_executor_)
            ur_instruction_executor_->cancelMotion();

        if (tool_update_failed_.load())
        {
            tcp_verifier_->waitForChange(std::chrono::milliseconds(100));
            continue;
        }

        if (!ur_program_connected_.load() || !robot_state_manager_ ||
            !robot_state_manager_->isReady() || !tcp_verifier_->programRunning())
        {
            tcp_verifier_->waitForChange(std::chrono::milliseconds(100));
            continue;
        }

        const std::uint64_t epoch = tcp_verifier_->claimPendingEpoch();
        if (!epoch)
        {
            tcp_verifier_->waitForChange(std::chrono::milliseconds(100));
            continue;
        }

        std::unique_lock<std::mutex> operation_lock(config_operation_mutex_);
        if (tool_update_failed_.load() || tcp_verifier_->epoch() != epoch || tcp_verifier_->isFault() ||
            tcp_verifier_->isVerified() ||
            !tcp_verifier_->prepareReconfiguration(kinematics_manager_->expectedTcpOffset(), epoch))
            continue;

        ROS_INFO_STREAM("[TCP]: Applying TCP and payload for program epoch " << epoch);
        try
        {
            kinematics_manager_->applyRobotConfig();
            const std::uint64_t after_sequence = tcp_verifier_->sequence();
            if (!tcp_verifier_->beginVerification(epoch, after_sequence))
                continue;

            const auto result = tcp_verifier_->waitForVerification(epoch,
                std::chrono::milliseconds(static_cast<int>(tcp_verification_timeout_s_ * 1000.0)));
            if (result == TcpVerifier::WaitResult::VERIFIED)
            {
                kinematics_manager_->markRobotConfigVerified();
                BridgeRtdeSample verified_sample;
                if (tcp_verifier_->snapshot(verified_sample))
                    ROS_INFO_STREAM("[TCP]: Controller readback verified for program epoch " << epoch
                        << " expected_tcp_offset=" << format_tcp(tcp_verifier_->expected())
                        << " latest_rtde_tcp_offset=" << format_tcp(verified_sample.tcp_offset)
                        << " latest_controller_timestamp=" << std::setprecision(17)
                        << verified_sample.controller_timestamp << " (translation in m, rotation vector in rad)");
                else
                    ROS_WARN_STREAM("[TCP]: Verification succeeded without an RTDE snapshot for program epoch " << epoch);
            }
            else if (result == TcpVerifier::WaitResult::TIMEOUT)
            {
                tcp_verifier_->fail(epoch, "TCP readback did not match before timeout");
                ROS_ERROR_STREAM("[TCP]: Verification failed: "
                    << format_tcp_state(tcp_verifier_->faultReason()));
            }
            else if (result == TcpVerifier::WaitResult::FAULT)
                ROS_ERROR_STREAM("[TCP]: Verification failed: "
                    << format_tcp_state(tcp_verifier_->faultReason()));
        }
        catch (const std::exception& e)
        {
            tcp_verifier_->fail(epoch, e.what());
            ROS_ERROR_STREAM("[TCP]: Configuration failed: " << e.what());
        }
    }
}

void shutdown(std::string reason)
{
    ROS_WARN_STREAM("Shutting down node, reason: " << reason);
    ros::requestShutdown();
    if (spinner_) spinner_->stop();
    bridge_threads_running_.store(false);
    config_worker_running_.store(false);
    if (config_thread_.joinable()) config_thread_.join();
    if (rtde_thread_.joinable()) rtde_thread_.join();
    if (keep_alive_thread_.joinable()) keep_alive_thread_.join();
    if (robot_state_manager_) robot_state_manager_->stop();
    // Drain any in-flight URCL callback before dashboard or driver teardown.
    {
        std::lock_guard<std::mutex> callback_lock(program_callback_mutex_);
        program_callback_enabled_ = false;
        ur_program_connected_.store(false);
    }
    if (nh_ != nullptr)
    {
        motor_off_on_shutdown_ = nh_->param("motor_off_for_deactivation", true);
        nh_->shutdown();
    }

    if (ur_primary_)
    {
        cleanup_step("primary commandStop", [] { ur_primary_->commandStop(); });
        cleanup_step("primary stop", [] { ur_primary_->stop(); });
    }

    if (ur_dashboard_)
    {
        if (!local_control_detected_at_startup)
        {
            cleanup_step("dashboard clear operational mode", [] {
                if (!ur_dashboard_->commandClearOperationalMode())
                    ROS_WARN("[Shutdown]: Dashboard did not clear operational mode");
            });
            if (motor_off_on_shutdown_)
                cleanup_step("dashboard power off", [] {
                    if (!ur_dashboard_->commandPowerOff())
                        ROS_WARN("[Shutdown]: Dashboard did not confirm power off");
                });
        }
        else
            ROS_INFO("[Shutdown]: Skipping dashboard state commands after Local Control was detected at startup");
        cleanup_step("dashboard disconnect", [] { ur_dashboard_->disconnect(); });
    }

    if (ur_driver_)
        cleanup_step("driver stopControl", [] { ur_driver_->stopControl(); });

    // Release every shared driver owner before destroying the verifier.
    // Late URCL program callbacks now return early.
    robot_state_manager_.reset();
    io_manager_.reset();
    kinematics_manager_.reset();
    ur_instruction_executor_.reset();
    ur_primary_.reset();
    ur_driver_.reset();
    tcp_verifier_.reset();
    ur_dashboard_.reset();

    ur_driver::unregisterUrclLogHandler();
    urcl::setLogLevel(urcl::LogLevel::INFO);
}

bool exec_traj(std::vector<std::shared_ptr<urcl::control::MotionPrimitive>> waypoints)
{
    if (!motion_ready())
        return false;

    std::lock_guard<std::mutex> send_command_lock_guard(send_command_mutex_);
    if (!motion_ready())
        return false;
    return ur_instruction_executor_->executeMotion(waypoints);
}

bool move_l(std::vector<geometry_msgs::Pose> waypoints, std::vector<double> velocities, std::vector<double> accelerations, std::vector<double> blending_radiuses)
{
    std::vector<std::shared_ptr<urcl::control::MotionPrimitive>> targets;
    KDL::Rotation rot;
    urcl::Pose pose;
    double vel, acc;

    for (long unsigned int i = 0; i < waypoints.size(); i++)
    {
        rot = KDL::Rotation::Quaternion(waypoints[i].orientation.x, waypoints[i].orientation.y, waypoints[i].orientation.z, waypoints[i].orientation.w);
        pose.x = waypoints[i].position.x;
        pose.y = waypoints[i].position.y;
        pose.z = waypoints[i].position.z;
        pose.rx = rot.GetRot().x();
        pose.ry = rot.GetRot().y();
        pose.rz = rot.GetRot().z();
        vel = max_speed_linear_ * velocities[i];
        acc = max_acceleration_linear_ * accelerations[i];

        targets.push_back(std::make_shared<urcl::control::MoveLPrimitive>(pose, blending_radiuses[i], std::chrono::milliseconds(0), acc, vel));
    }

    return exec_traj(targets);
}

bool move_j(std::vector<KDL::JntArray> waypoints, std::vector<double> velocities, std::vector<double> accelerations, std::vector<double> blending_radiuses)
{
    std::vector<std::shared_ptr<urcl::control::MotionPrimitive>> targets;
    urcl::vector6d_t joints;
    double vel, acc;

    for (long unsigned int i = 0; i < waypoints.size(); i++)
    {
        for (int j = 0; j < kinematics_manager_->getNumJoints(); j++)
        {
            joints[j] = waypoints[i](j);
        }
        vel = max_speed_joint_ * velocities[i];
        acc = max_acceleration_joint_ * accelerations[i];

        targets.push_back(std::make_shared<urcl::control::MoveJPrimitive>(joints, blending_radiuses[i], std::chrono::milliseconds(0), acc, vel));
    }

    return exec_traj(targets);
}

// -----------------------------------------
//           Services Callbacks
// -----------------------------------------

bool callback_check_hardware_status(rpwc_msgs::checkHardwareStatus::Request &req, rpwc_msgs::checkHardwareStatus::Response &res)
{
    bool init_success;
    std::string init_message;
    {
        std::lock_guard<std::mutex> lock(startup_state_mutex);
        res.status.data = init_status_;
        init_success = init_success_;
        init_message = init_msg_;
    }
    res.result.data = init_success && motion_ready();
    if (init_success && !res.result.data)
    {
        res.info.data = tool_update_failed_.load() ?
            "Tool update failed; deactivate the arm and verify the physical tool before reactivation" :
            tcp_verifier_ && tcp_verifier_->isFault() ?
            "TCP verification failed: " + tcp_verifier_->faultReason() :
            "Robot program, TCP verification, or RTDE stream not ready";
    }
    else
        res.info.data = init_message;
    return true;
}

bool callback_set_tcp_payload(rpwc_msgs::setTcpPayload::Request &req, rpwc_msgs::setTcpPayload::Response &res)
{
    const auto reject = [&res](const std::string& reason) {
        res.result.data = false;
        res.info.data = reason;
        return true;
    };
    if (!kinematics_manager_ || !tcp_verifier_ || !robot_state_manager_)
        return reject("Robot configuration is unavailable");
    if (tool_update_failed_.load())
        return reject("A previous tool update failed; deactivate the arm before retrying");

    KinematicsManager::RobotConfig requested;
    try
    {
        requested = kinematics_manager_->makeRobotConfig(
            {{req.tcp_translation.x, req.tcp_translation.y, req.tcp_translation.z}},
            {{req.tcp_rpy.x, req.tcp_rpy.y, req.tcp_rpy.z}}, req.mass.data,
            {{req.center_of_mass.x, req.center_of_mass.y, req.center_of_mass.z}});
    }
    catch (const std::exception& e)
    {
        return reject(e.what());
    }

    std::unique_lock<std::mutex> operation_lock(config_operation_mutex_, std::try_to_lock);
    if (!operation_lock.owns_lock())
        return reject("Robot configuration is already in progress");
    if (tool_update_failed_.load())
        return reject("A previous tool update failed; deactivate the arm before retrying");
    {
        std::lock_guard<std::mutex> admission_lock(goal_admission_mutex_);
        if (active_motion_goal_.load())
            return reject("A robot motion goal is active");
        if (reconfiguring_robot_.load())
            return reject("Robot configuration is already in progress");
        std::lock_guard<std::mutex> command_lock(send_command_mutex_);
        if (freedrive_)
            return reject("Freedrive is active");
        if (!ur_program_connected_.load() || !robot_state_manager_->isReady() ||
            !tcp_verifier_->programRunning())
            return reject("UR program or RTDE is not ready");
        reconfiguring_robot_.store(true);
    }
    ReconfigurationScope reconfiguration_scope;
    const auto fail_update = [&reject](const std::string& reason) {
        tool_update_failed_.store(true);
        cancel_required_.store(true);
        ROS_ERROR_STREAM("[TCP]: Tool update failed; motion stays blocked until arm reactivation: " << reason);
        return reject(reason);
    };
    kinematics_manager_->invalidateRobotConfig();

    const auto previous_expected = tcp_verifier_->expected();
    auto config_lock = kinematics_manager_->lockRobotConfig();
    const std::uint64_t epoch = tcp_verifier_->epoch();
    if (!tcp_verifier_->prepareReconfiguration(requested.tcp_offset, epoch))
    {
        tcp_verifier_->failReconfiguration(epoch, previous_expected,
            "UR program or RTDE became unavailable during reconfiguration");
        return fail_update("UR program or RTDE became unavailable during reconfiguration");
    }

    try
    {
        kinematics_manager_->sendRobotConfig(requested, config_lock);
    }
    catch (const std::exception& e)
    {
        tcp_verifier_->failReconfiguration(epoch, previous_expected, e.what());
        ROS_ERROR_STREAM("[TCP]: Runtime configuration failed: " << e.what());
        return fail_update(e.what());
    }

    const std::uint64_t after_sequence = tcp_verifier_->sequence();
    if (!tcp_verifier_->beginVerification(epoch, after_sequence))
    {
        tcp_verifier_->failReconfiguration(epoch, previous_expected,
            "UR program or RTDE changed before TCP verification");
        return fail_update("UR program or RTDE changed before TCP verification");
    }

    const auto verification = tcp_verifier_->waitForVerification(epoch,
        std::chrono::milliseconds(static_cast<int>(tcp_verification_timeout_s_ * 1000.0)));
    if (verification != TcpVerifier::WaitResult::VERIFIED ||
        !ur_program_connected_.load() || !robot_state_manager_->isReady() ||
        !tcp_verifier_->motionReady() || tcp_verifier_->epoch() != epoch)
    {
        const std::string reason = verification == TcpVerifier::WaitResult::TIMEOUT ?
            "TCP readback did not match before timeout" :
            verification == TcpVerifier::WaitResult::FAULT ?
            tcp_verifier_->faultReason() :
            "UR program or RTDE changed during TCP verification";
        tcp_verifier_->failReconfiguration(epoch, previous_expected, reason);
        ROS_ERROR_STREAM("[TCP]: Runtime configuration failed: " << reason);
        return fail_update(reason);
    }

    try
    {
        kinematics_manager_->commitVerifiedRobotConfig(requested, config_lock);
    }
    catch (const std::exception& e)
    {
        tcp_verifier_->failReconfiguration(epoch, previous_expected, e.what());
        return fail_update(std::string("Failed to commit verified tool configuration: ") + e.what());
    }
    ROS_INFO_STREAM("[TCP]: Runtime controller TCP verified: "
        << format_tcp(tcp_verifier_->expected()) << "; payload command accepted");
    res.result.data = true;
    res.info.data = "TCP verified from RTDE; payload command accepted";
    return true;
}

bool callback_set_controller(rpwc_msgs::setController::Request &req, rpwc_msgs::setController::Response &res)
{
    if (req.controller != 2 && last_controller_started_ != 2)
    {
        if (req.controller != 99)
            last_controller_started_ = req.controller;
        res.result.data = true;
        return true;
    }
    else if (req.controller != 2)
    {
        ROS_INFO("Start native controller");
        std::lock_guard<std::mutex> lock(send_command_mutex_);
        if (reconfiguring_robot_.load())
        {
            res.result.data = false;
            return true;
        }
        if (req.controller != 99)
            last_controller_started_ = req.controller;
        else
            last_controller_started_ = 0;

        res.result.data = ur_driver_->writeFreedriveControlMessage(urcl::control::FreedriveControlMessage::FREEDRIVE_STOP);
        freedrive_ = false;
        return true;
    }

    ROS_INFO("Start freedrive");
    std::lock_guard<std::mutex> lock(send_command_mutex_);
    if (reconfiguring_robot_.load())
    {
        res.result.data = false;
        return true;
    }
    last_controller_started_ = 2;
    freedrive_ = true;
    res.result.data = ur_driver_->writeFreedriveControlMessage(urcl::control::FreedriveControlMessage::FREEDRIVE_START, freedrive_params_, urcl::RobotReceiveTimeout::millisec(200));
    return true;
}

bool callback_get_controller(rpwc_msgs::getController::Request &req, rpwc_msgs::getController::Response &res)
{
    res.controller = last_controller_started_;
    return true;
}

bool callback_set_free_jog_params(rpwc_msgs::setFreeJogParams::Request &req, rpwc_msgs::setFreeJogParams::Response &res)
{
    switch (req.mode.data)
    {
        case 0: // Joints
            freedrive_params_.lock_x = false;
            freedrive_params_.lock_y = false;
            freedrive_params_.lock_z = false;
            freedrive_params_.lock_rx = false;
            freedrive_params_.lock_ry = false;
            freedrive_params_.lock_rz = false;
            freedrive_params_.ref_frame = urcl::control::FreedriveReferenceFrame::BASE;
            break;

        case 1: // Cartesian
            freedrive_params_.lock_x = req.lock_x.data;
            freedrive_params_.lock_y = req.lock_y.data;
            freedrive_params_.lock_z = req.lock_z.data;
            freedrive_params_.lock_rx = req.lock_rx.data;
            freedrive_params_.lock_ry = req.lock_ry.data;
            freedrive_params_.lock_rz = req.lock_rz.data;
            freedrive_params_.ref_frame = urcl::control::FreedriveReferenceFrame(req.ref_frame.data);
            break;

        default:
            ROS_ERROR_STREAM("Unknow Free Jog mode: " << req.mode.data);
            res.success.data = false;
            return true;
    }

    freedrive_params_.mode = urcl::control::FreedriveMode(req.mode.data);

    if (freedrive_)
    {
        std::lock_guard<std::mutex> lock(send_command_mutex_);
        if (!ur_driver_->writeFreedriveControlMessage(urcl::control::FreedriveControlMessage::FREEDRIVE_STOP))
        {
            res.success.data = false;
            res.info.data = "Failed to disable Free Jog";
            return true;
        }

        if (!ur_driver_->writeFreedriveControlMessage(urcl::control::FreedriveControlMessage::FREEDRIVE_START, freedrive_params_, urcl::RobotReceiveTimeout::millisec(200)))
        {
            res.success.data = false;
            res.info.data = "Failed to enable Free Jog";
            return true;
        }
    }

    res.success.data = true;
    return true;
}

bool callback_get_free_jog_params(rpwc_msgs::getFreeJogParams::Request &req, rpwc_msgs::getFreeJogParams::Response &res)
{
    res.ref_frame.data = urcl::toUnderlying(freedrive_params_.ref_frame);
    res.mode.data = urcl::toUnderlying(freedrive_params_.mode);
    res.lock_x.data = freedrive_params_.lock_x;
    res.lock_y.data = freedrive_params_.lock_y;
    res.lock_z.data = freedrive_params_.lock_z;
    res.lock_rx.data = freedrive_params_.lock_rx;
    res.lock_ry.data = freedrive_params_.lock_ry;
    res.lock_rz.data = freedrive_params_.lock_rz;

    return true;
}

bool callback_set_speed_override(rpwc_msgs::setSpeedOverride::Request &req, rpwc_msgs::setSpeedOverride::Response &res)
{
    if (req.ratio.data <= 0.0 || req.ratio.data > 1.0)
    {
        res.success.data = false;
        res.info.data = "Speed override should be a value between 0.0 (excluded) and 1.0 (included)";
        return true;
    }

    speed_override_ = req.ratio.data;
    res.success.data = ur_driver_->getRTDEWriter().sendSpeedSlider(speed_override_);

    if (!res.success.data)
        res.info.data = "Failed to set speed override, check log for more info";

    return true;
}

bool callback_get_speed_override(rpwc_msgs::getSpeedOverride::Request &req, rpwc_msgs::getSpeedOverride::Response &res)
{
    res.ratio.data = speed_override_;
    res.success.data = true;
    return true;
}

// -----------------------------------------
//             Actions Servers
// -----------------------------------------

// Cartesian Action Server
CartesianMove::CartesianMove(std::string name) : as(*nh_, name, false)
{
    as.registerGoalCallback(boost::bind(&CartesianMove::goal_callback, this));
    as.registerPreemptCallback(boost::bind(&CartesianMove::preempt_callback, this));

    as.start();
}

CartesianMove::~CartesianMove(void)
{
    if (as.isActive())
    {
        rpwc_msgs::nativeCartesianCommandsResult result;
        result.success = false;
        result.error_code = 9;
        result.msg = "Node shutdown";
        as.setAborted(result, result.msg);
    }
    as.shutdown();
}

void CartesianMove::goal_callback()
{
    ROS_INFO("[Cartesian Move]: Accepting new goal");
    const auto rpwc_goal = as.acceptNewGoal();
    rpwc_msgs::nativeCartesianCommandsResult rpwc_result;
    bool configuration_in_progress = false;
    if (!try_claim_motion_goal(configuration_in_progress))
    {
        // SimpleActionServer may already have preempted the previous action
        // when acceptNewGoal() returned. Stop its physical motion before
        // rejecting this overlapping goal.
        if (!configuration_in_progress && ur_instruction_executor_ &&
            !ur_instruction_executor_->cancelMotion() &&
            robot_state_manager_ && !robot_state_manager_->forceProgramStop())
            ROS_FATAL("[Cartesian Move]: Could not confirm cancellation of overlapping motion");
        rpwc_result.success = false;
        rpwc_result.error_code = 9;
        rpwc_result.msg = configuration_in_progress ?
            "Robot configuration in progress" : "Another robot motion is active";
        as.setAborted(rpwc_result, rpwc_result.msg);
        return;
    }
    MotionGoalScope motion_goal_scope;

    if (!robot_state_manager_ || !robot_state_manager_->isReady())
    {
        if (robot_state_manager_ && robot_state_manager_->isSafeguardActive())
        {
            ROS_WARN("[Cartesian Move]: Safeguard stop active, waiting for clearance");
            while (!robot_state_manager_->isReady() && !as.isPreemptRequested() && ros::ok())
                ros::Duration(0.1).sleep();

            if (as.isPreemptRequested())
            {
                as.setPreempted();
                return;
            }
        }

        if (!robot_state_manager_ || !robot_state_manager_->isReady())
        {
            ROS_WARN("[Cartesian Move]: Robot program not ready, aborting goal");
            rpwc_result.success = false;
            rpwc_result.error_code = 9;
            rpwc_result.msg = "Robot program not ready";
            as.setAborted(rpwc_result, rpwc_result.msg);
            return;
        }
    }

    if (!motion_ready())
    {
        rpwc_result.success = false;
        rpwc_result.error_code = 9;
        rpwc_result.msg = tcp_verifier_ && tcp_verifier_->isFault() ?
            "TCP verification failed: " + tcp_verifier_->faultReason() : "TCP not verified or RTDE stale";
        ROS_WARN_STREAM("[TCP]: Cartesian goal rejected: " << format_tcp_state(rpwc_result.msg));
        as.setAborted(rpwc_result, rpwc_result.msg);
        return;
    }

    const std::size_t len = rpwc_goal->Poses.size();
    if (rpwc_goal->types.size() != len || rpwc_goal->velocities.size() != len ||
        rpwc_goal->accelerations.size() != len || rpwc_goal->zone_radiuses.size() != len)
    {
        rpwc_result.success = false;
        rpwc_result.error_code = 9;
        rpwc_result.msg = "Cartesian goal array lengths differ";
        as.setAborted(rpwc_result, rpwc_result.msg);
        return;
    }
    if (!finite_vector(rpwc_goal->velocities) || !finite_vector(rpwc_goal->accelerations) ||
        !finite_vector(rpwc_goal->zone_radiuses) ||
        !std::all_of(rpwc_goal->Poses.begin(), rpwc_goal->Poses.end(), finite_pose))
    {
        rpwc_result.success = false;
        rpwc_result.error_code = 9;
        rpwc_result.msg = "Cartesian goal contains nonfinite values";
        as.setAborted(rpwc_result, rpwc_result.msg);
        return;
    }

    ROS_INFO_STREAM("[Cartesian Move]: Max valid poses: " << len);
    if (len == 0)
    {
        ROS_WARN("[Cartesian Move]: No valid poses found, aborting");
        rpwc_result.success = false;
        rpwc_result.error_code = 9;
        rpwc_result.msg = "No data found";
        as.setAborted(rpwc_result, rpwc_result.msg);
        return;
    }

    for (auto &type : rpwc_goal->types)
    {
        if (type != rpwc_goal->LINEAR_MOVE)
        {
            ROS_ERROR_STREAM("[Cartesian Move]: Uknown move type: '" << type << "' aborting goal");
            rpwc_result.success = false;
            rpwc_result.error_code = 9;
            rpwc_result.msg = "Uknown move type";
            as.setAborted(rpwc_result, rpwc_result.msg);
            return;
        }
    }

    ROS_INFO("[Cartesian Move]: Executing trajectory");
    rpwc_result.success = move_l(rpwc_goal->Poses, rpwc_goal->velocities, rpwc_goal->accelerations, rpwc_goal->zone_radiuses);

    if (!rpwc_result.success)
    {
        ROS_INFO("[Cartesian Move]: Goal aborted");
        rpwc_result.error_code = 9;
        rpwc_result.msg = "Goal failed";
        as.setAborted(rpwc_result, rpwc_result.msg);
        return;
    }

    ROS_INFO("[Cartesian Move]: Goal completed");
    rpwc_result.msg = "Goal succeded";
    as.setSucceeded(rpwc_result, rpwc_result.msg);
}

void CartesianMove::preempt_callback()
{
    ROS_INFO("[Cartesian Move]: Goal preempted");

    if (robot_state_manager_ && robot_state_manager_->isSafeguardActive())
    {
        ROS_WARN("[Cartesian Move]: Safeguard active, forcing program stop to cancel motion");
        if (robot_state_manager_->forceProgramStop())
        {
            as.setPreempted();
        }
        else
        {
            ROS_ERROR("[Cartesian Move]: Failed to force-stop robot program, falling back to cancelMotion");
            ur_instruction_executor_->cancelMotion();
            rpwc_msgs::nativeCartesianCommandsResult result;
            result.success = false;
            result.error_code = 9;
            result.msg = "Failed to confirm cancellation during safeguard stop";
            as.setAborted(result, result.msg);
        }
    }
    else
    {
        ur_instruction_executor_->cancelMotion();
        as.setPreempted();
    }
}

// Joints Action Server
JointsMove::JointsMove(std::string name) : as(*nh_, name, false)
{
    as.registerGoalCallback(boost::bind(&JointsMove::goal_callback, this));
    as.registerPreemptCallback(boost::bind(&JointsMove::preempt_callback, this));

    as.start();
}

JointsMove::~JointsMove(void)
{
    if (as.isActive())
    {
        rpwc_msgs::nativeJointsCommandsResult result;
        result.success = false;
        result.error_code = 9;
        result.msg = "Node shutdown";
        as.setAborted(result, result.msg);
    }
    as.shutdown();
}

void JointsMove::goal_callback()
{
    ROS_INFO("[Joints Move]: Accepting new goal");
    const auto rpwc_goal = as.acceptNewGoal();
    rpwc_msgs::nativeJointsCommandsResult rpwc_result;
    bool configuration_in_progress = false;
    if (!try_claim_motion_goal(configuration_in_progress))
    {
        if (!configuration_in_progress && ur_instruction_executor_ &&
            !ur_instruction_executor_->cancelMotion() &&
            robot_state_manager_ && !robot_state_manager_->forceProgramStop())
            ROS_FATAL("[Joints Move]: Could not confirm cancellation of overlapping motion");
        rpwc_result.success = false;
        rpwc_result.error_code = 9;
        rpwc_result.msg = configuration_in_progress ?
            "Robot configuration in progress" : "Another robot motion is active";
        as.setAborted(rpwc_result, rpwc_result.msg);
        return;
    }
    MotionGoalScope motion_goal_scope;

    if (!robot_state_manager_ || !robot_state_manager_->isReady())
    {
        if (robot_state_manager_ && robot_state_manager_->isSafeguardActive())
        {
            ROS_WARN("[Joints Move]: Safeguard stop active, waiting for clearance");
            while (!robot_state_manager_->isReady() && !as.isPreemptRequested() && ros::ok())
                ros::Duration(0.1).sleep();

            if (as.isPreemptRequested())
            {
                as.setPreempted();
                return;
            }
        }

        if (!robot_state_manager_ || !robot_state_manager_->isReady())
        {
            ROS_WARN("[Joints Move]: Robot program not ready, aborting goal");
            rpwc_result.success = false;
            rpwc_result.error_code = 9;
            rpwc_result.msg = "Robot program not ready";
            as.setAborted(rpwc_result, rpwc_result.msg);
            return;
        }
    }

    if (!motion_ready())
    {
        rpwc_result.success = false;
        rpwc_result.error_code = 9;
        rpwc_result.msg = tcp_verifier_ && tcp_verifier_->isFault() ?
            "TCP verification failed: " + tcp_verifier_->faultReason() : "TCP not verified or RTDE stale";
        ROS_WARN_STREAM("[TCP]: Joint goal rejected: " << format_tcp_state(rpwc_result.msg));
        as.setAborted(rpwc_result, rpwc_result.msg);
        return;
    }

    const std::size_t len = rpwc_goal->targets.size();
    if (rpwc_goal->velocities.size() != len || rpwc_goal->accelerations.size() != len ||
        rpwc_goal->zone_radiuses.size() != len)
    {
        rpwc_result.success = false;
        rpwc_result.error_code = 9;
        rpwc_result.msg = "Joint goal array lengths differ";
        as.setAborted(rpwc_result, rpwc_result.msg);
        return;
    }
    if (!finite_vector(rpwc_goal->velocities) || !finite_vector(rpwc_goal->accelerations) ||
        !finite_vector(rpwc_goal->zone_radiuses))
    {
        rpwc_result.success = false;
        rpwc_result.error_code = 9;
        rpwc_result.msg = "Joint goal contains nonfinite speed or blend values";
        as.setAborted(rpwc_result, rpwc_result.msg);
        return;
    }

    ROS_INFO_STREAM("[Joints Move]: Max valid poses: " << len);
    if (len == 0)
    {
        ROS_WARN("[Joints Move]: No valid poses found, aborting");
        rpwc_result.success = false;
        rpwc_result.error_code = 9;
        rpwc_result.msg = "No data found";
        as.setAborted(rpwc_result, rpwc_result.msg);
        return;
    }

    std::vector<KDL::JntArray> waypoints;
    KDL::JntArray tmpWaypoint;
    tmpWaypoint.resize(kinematics_manager_->getNumJoints());
    for (auto it = rpwc_goal->targets.begin(); it != rpwc_goal->targets.end(); it++)
    {
        if (it->values.size() != static_cast<std::size_t>(kinematics_manager_->getNumJoints()))
        {
            rpwc_result.success = false;
            rpwc_result.error_code = 9;
            rpwc_result.msg = "Joint goal has incorrect joint count";
            as.setAborted(rpwc_result, rpwc_result.msg);
            return;
        }
        if (!finite_vector(it->values))
        {
            rpwc_result.success = false;
            rpwc_result.error_code = 9;
            rpwc_result.msg = "Joint goal contains nonfinite target";
            as.setAborted(rpwc_result, rpwc_result.msg);
            return;
        }
        for (int i = 0; i < kinematics_manager_->getNumJoints(); i++)
        {
            tmpWaypoint(i) = it->values[i];
        }
        waypoints.push_back(tmpWaypoint);
    }

    ROS_INFO("[Joints Move]: Executing trajectory");
    rpwc_result.success = move_j(waypoints, rpwc_goal->velocities, rpwc_goal->accelerations, rpwc_goal->zone_radiuses);

    if (!rpwc_result.success)
    {
        ROS_INFO("[Joints Move]: Goal aborted");
        rpwc_result.error_code = 9;
        rpwc_result.msg = "Goal failed";
        as.setAborted(rpwc_result, rpwc_result.msg);
        return;
    }

    ROS_INFO("[Joints Move]: Goal completed");
    rpwc_result.msg = "Goal succeded";
    as.setSucceeded(rpwc_result, rpwc_result.msg);
}

void JointsMove::preempt_callback()
{
    ROS_INFO("[Joints Move]: Goal preempted");

    if (robot_state_manager_ && robot_state_manager_->isSafeguardActive())
    {
        ROS_WARN("[Joints Move]: Safeguard active, forcing program stop to cancel motion");
        if (robot_state_manager_->forceProgramStop())
        {
            as.setPreempted();
        }
        else
        {
            ROS_ERROR("[Joints Move]: Failed to force-stop robot program, falling back to cancelMotion");
            ur_instruction_executor_->cancelMotion();
            rpwc_msgs::nativeJointsCommandsResult result;
            result.success = false;
            result.error_code = 9;
            result.msg = "Failed to confirm cancellation during safeguard stop";
            as.setAborted(result, result.msg);
        }
    }
    else
    {
        ur_instruction_executor_->cancelMotion();
        as.setPreempted();
    }
}

// -----------------------------------------
//                  Main
// -----------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rpwc_ur_bridge_native");
    nh_ = new ros::NodeHandle();
    spinner_.reset(new ros::AsyncSpinner(4));
    spinner_->start();

    // Set default 
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) // Debug
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    // Print urcl logs using ros logs
    ur_driver::registerUrclLogHandler();

    // Init variables
    last_controller_started_ = 0;
    freedrive_ = false;
    freedrive_params_ = {};
    speed_override_ = 1.0;
    name_space_ = nh_->getNamespace();
    motor_off_on_shutdown_ = true;
    local_control_detected_at_startup = false;
    {
        std::lock_guard<std::mutex> lock(startup_state_mutex);
        init_status_ = 0;
        init_success_ = false;
        init_msg_.clear();
        startup_phase = StartupPhase::STARTING;
        startup_rtde_failure.clear();
    }

    // Start service for init feedback
    ros::ServiceServer check_hardware_status_srv = nh_->advertiseService<rpwc_msgs::checkHardwareStatus::RequestType, rpwc_msgs::checkHardwareStatus::ResponseType>("check_hardware_status", &callback_check_hardware_status);

    // Load params
    if (!nh_->getParam("robot_ip", robot_ip_))
    {
        ROS_FATAL_STREAM("Param '" << name_space_ << "/robot_ip' not found on param server");
        set_init_end_status(false, "Param robot_ip missing");
        return 1;
    }

    if (!nh_->getParam("urscript_file", urscript_file_path_))
    {
        ROS_ERROR_STREAM("Param '" << name_space_ << "/urscript_file' not found on param server");
        set_init_end_status(false, "Param urscript_file missing");
        return 1;
    }

    if (!nh_->getParam("kinematics/hash", calibration_hash_))
    {
        ROS_ERROR_STREAM("Param '" << name_space_ << "/kinematics/hash' not found on param server");
        set_init_end_status(false, "Param kinematics/hash missing");
        return 1;
    }

    if (!nh_->getParam("max_speed_linear", max_speed_linear_))
    {
        ROS_ERROR_STREAM("Param '" << name_space_ << "/max_speed_linear' not found on param server");
        set_init_end_status(false, "Param max_speed_linear missing");
        return 1;
    }

    if (!nh_->getParam("max_acceleration_linear", max_acceleration_linear_))
    {
        ROS_ERROR_STREAM("Param '" << name_space_ << "/max_acceleration_linear' not found on param server");
        set_init_end_status(false, "Param max_acceleration_linear missing");
        return 1;
    }

    if (!nh_->getParam("max_speed_joint", max_speed_joint_))
    {
        ROS_ERROR_STREAM("Param '" << name_space_ << "/max_speed_joint' not found on param server");
        set_init_end_status(false, "Param max_speed_joint missing");
        return 1;
    }

    if (!nh_->getParam("max_acceleration_joint", max_acceleration_joint_))
    {
        ROS_ERROR_STREAM("Param '" << name_space_ << "/max_acceleration_joint' not found on param server");
        set_init_end_status(false, "Param max_acceleration_joint missing");
        return 1;
    }

    nh_->param<double>("rate_rtde_hz", freq_rtde_hz_, 50.0);
    nh_->param<double>("tcp_verification_timeout_s", tcp_verification_timeout_s_, 2.0);
    double startup_tcp_timeout_s = 20.0;
    nh_->param<double>("startup_tcp_timeout_s", startup_tcp_timeout_s, startup_tcp_timeout_s);
    if (!std::isfinite(freq_rtde_hz_) || freq_rtde_hz_ <= 0.0 ||
        !std::isfinite(tcp_verification_timeout_s_) || tcp_verification_timeout_s_ <= 0.0 ||
        !std::isfinite(startup_tcp_timeout_s) || startup_tcp_timeout_s <= 0.0 ||
        !std::isfinite(max_speed_linear_) || !std::isfinite(max_acceleration_linear_) ||
        !std::isfinite(max_speed_joint_) || !std::isfinite(max_acceleration_joint_))
    {
        set_init_end_status(false, "Invalid RTDE, TCP verification, or motion scale parameter");
        return 1;
    }

    bool auto_recover_protective_stop;
    double recovery_timeout_s;
    int recovery_retries;
    nh_->param<bool>("auto_recover_protective_stop", auto_recover_protective_stop, false);
    nh_->param<double>("recovery_timeout_s", recovery_timeout_s, 3.0);
    nh_->param<int>("recovery_retries", recovery_retries, 3);

    // Use dashboard server to prepare robot controller
    ROS_INFO("Starting Dashboard");
    ur_dashboard_.reset(new urcl::DashboardClient(robot_ip_));
    if (!ur_dashboard_->connect(3, std::chrono::seconds(5)))
    {
        URCL_LOG_ERROR("Could not connect to dashboard");
        set_init_end_status(false, "Could not connect to dashboard");
        return 1;
    }

    timeval timeout;
    timeout.tv_sec = 3;
    timeout.tv_usec = 0;
    ur_dashboard_->setReceiveTimeout(timeout);

    // Start robot, if possible keep current status
    try
    {
        if (!ur_dashboard_->commandIsInRemoteControl())
        {
            local_control_detected_at_startup = true;
            ROS_ERROR("Robot controller must be in 'remote control' mode for this driver to work");
            set_init_end_status(false, "Robot controller must be in 'remote control' mode for this driver to work");
            return 1;
        }
    }
    catch (const urcl::UrException &e)
    {
        ROS_ERROR_STREAM("This driver does not support CB3 robots;\n"
                         << e.what());
        set_init_end_status(false, "CB3 robots are not supported");
        return 1;
    }

    try
    {
        if (check_safety_mode(urcl::SafetyMode::SYSTEM_EMERGENCY_STOP) || check_safety_mode(urcl::SafetyMode::ROBOT_EMERGENCY_STOP))
        {
            ROS_FATAL("Robot is in emergency stop");
            set_init_end_status(false, "Robot is in emergency stop");
            return 1;
        }

        if (check_robot_mode(urcl::RobotMode::CONFIRM_SAFETY))
        {
            ROS_FATAL("Robot remote control not possible, acknowledge safety erros from teach pendant");
            set_init_end_status(false, "Robot remote control not possible, acknowledge safety erros from teach pendant");
            return 1;
        }

        if (check_robot_mode(urcl::RobotMode::POWER_OFF))
        {
            if (!ur_dashboard_->commandPowerOn())
            {
                ROS_FATAL("Failed to power up robot");
                set_init_end_status(false, "Failed to power up robot");
                return 1;
            }
        }
    }
    catch (const std::exception &e)
    {
        ROS_ERROR_STREAM(e.what());
        set_init_end_status(false, "Unexpected error, check logs");
        return 1;
    }

    std::string op_mode;
    if (!ur_dashboard_->commandGetOperationalMode(op_mode))
    {
        ROS_ERROR("Failed to get operational mode");
        set_init_end_status(false, "Failed to get operational mode");
        return 1;
    }

    ROS_DEBUG_STREAM("Op mode: " << op_mode);
    if (op_mode != "AUTOMATIC")
    {
        if (!ur_dashboard_->commandClearOperationalMode())
        {
            ROS_ERROR("Failed to clear operational mode");
            set_init_end_status(false, "Failed to clear operational mode");
            return 1;
        }
    }

    ur_dashboard_->commandBrakeRelease();

    // If safeguard is active try to wait for unlock or timeout
    {
        ros::Time start = ros::Time::now();
        while (check_safety_mode(urcl::SafetyMode::SAFEGUARD_STOP))
        {
            if ((ros::Time::now() - start).sec >= 20)
            {
                ROS_FATAL("Safeguard Stop not released within timeout");
                set_init_end_status(false, "Safeguard Stop not released within timeout");
                return 1;
            }

            ros::Duration(0.5).sleep();
        }
    }

    // Start ur driver
    ROS_INFO("Create UR_Driver");
    urcl::UrDriverConfiguration urDriverConfig;
    urDriverConfig.robot_ip = robot_ip_;
    urDriverConfig.script_file = urscript_file_path_;
    urDriverConfig.output_recipe_file = ros::package::getPath("ur_robot_driver") + "/resources/rtde_output_recipe.txt";
    urDriverConfig.input_recipe_file = ros::package::getPath("ur_robot_driver") + "/resources/rtde_input_recipe.txt";
    urDriverConfig.headless_mode = true;
    urDriverConfig.handle_program_state = &handleRobotProgramState;

    try
    {
        ur_driver_.reset(new urcl::UrDriver(urDriverConfig));
        ur_driver_->resetRTDEClient(urDriverConfig.output_recipe_file,
            urDriverConfig.input_recipe_file, freq_rtde_hz_, true);
    }
    catch (const std::exception& e)
    {
        set_init_end_status(false, std::string("UR driver or RTDE initialization failed: ") + e.what());
        return 1;
    }
    if (!requiredRecipePresent(ur_driver_->getRTDEOutputRecipe()))
    {
        set_init_end_status(false, "Required RTDE fields unavailable");
        return 1;
    }
    ROS_INFO_STREAM("ControlFrequency: " << ur_driver_->getControlFrequency());

    // Init IO Manager
    std::string io_json_path;
    ros::param::get("io_signals_names_path", io_json_path);
    io_manager_.reset(new IOManager(*nh_, ur_driver_, io_json_path));

    // Build KDL and the expected controller TCP before RTDE can deliver a packet.
    try
    {
        kinematics_manager_.reset(new KinematicsManager(*nh_, ur_driver_));
    }
    catch (const std::exception &e)
    {
        set_init_end_status(false, e.what());
        return 1;
    }
    const auto expected = kinematics_manager_->expectedTcpOffset();
    const double max_age_seconds = std::max(0.25, 5.0 / freq_rtde_hz_);
    try
    {
        tcp_verifier_.reset(new TcpVerifier(expected,
            std::chrono::milliseconds(static_cast<int>(max_age_seconds * 1000.0))));
    }
    catch (const std::exception& e)
    {
        set_init_end_status(false, std::string("Invalid controller TCP configuration: ") + e.what());
        return 1;
    }
    ROS_INFO_STREAM("[TCP]: Expected controller tcp_offset=" << format_tcp(tcp_verifier_->expected())
        << " (translation in m, rotation vector in rad; after URCL serialization quantization)");

    // Init Robot State Manager
    robot_state_manager_.reset(new RobotStateManager(*nh_, ur_driver_, ur_dashboard_, auto_recover_protective_stop, recovery_timeout_s, recovery_retries));
    robot_state_manager_->setOnBlockedCallback([]() {
        if (ur_instruction_executor_)
            ur_instruction_executor_->cancelMotion();
    });
    bridge_threads_running_.store(true);
    rtde_thread_ = std::thread(&thread_handle_rtde);
    robot_state_manager_->start();

    bool calibValid = ur_driver_->checkCalibration(calibration_hash_);
    ROS_INFO_STREAM("checkCalibration: " << (calibValid ? "VALID" : "INVALID"));
    if (!calibValid)
    {
        ROS_ERROR_STREAM("The calibration parameters of the connected robot don't match the ones from the given kinematics "
                         "config file. Please be aware that this can lead to critical inaccuracies of tcp positions. Use "
                         "the ur_calibration tool to extract the correct calibration from the robot and pass that into the "
                         "description. See "
                         "[https://github.com/UniversalRobots/Universal_Robots_ROS_Driver#extract-calibration-information] "
                         "for details.");
    }

    ur_instruction_executor_.reset(new urcl::InstructionExecutor(ur_driver_));
    ur_primary_ = ur_driver_->getPrimaryClient();

    config_worker_running_.store(true);
    config_thread_ = std::thread(&thread_configure_robot);
    // Wait for program start — state manager handles recovery and shutdown on failure
    const auto startup_deadline = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(static_cast<int>(startup_tcp_timeout_s * 1000.0));
    while (!robot_state_manager_->isReady() && ros::ok() && pending_rtde_failure().empty() &&
           std::chrono::steady_clock::now() < startup_deadline)
        ros::Duration(0.05).sleep();

    if (!ros::ok() || !robot_state_manager_->isReady() || !pending_rtde_failure().empty())
    {
        const std::string rtde_error = pending_rtde_failure();
        const std::string message = rtde_error.empty() ?
            "Program or RTDE did not become ready before startup timeout" : rtde_error;
        ROS_FATAL_STREAM(message);
        set_init_end_status(false, message);
        return 1;
    }

    // The configuration worker applies TCP/payload and confirms the active TCP
    // from fresh controller RTDE packets before any motion action is advertised.
    while (ros::ok() && !motion_ready() && !tcp_verifier_->isFault() &&
           pending_rtde_failure().empty() &&
           std::chrono::steady_clock::now() < startup_deadline)
        ros::Duration(0.05).sleep();
    if (!ros::ok() || !motion_ready() || !pending_rtde_failure().empty())
    {
        const std::string rtde_error = pending_rtde_failure();
        const std::string message = !rtde_error.empty() ? rtde_error :
            (tcp_verifier_->isFault() ? "TCP verification failed: " + tcp_verifier_->faultReason() :
            "TCP was not verified before startup timeout");
        ROS_FATAL_STREAM(message);
        set_init_end_status(false, message);
        return 1;
    }
    kinematics_manager_->start(freq_rtde_hz_);

    if (!ur_driver_->getRTDEWriter().sendSpeedSlider(speed_override_))
    {
        ROS_FATAL("Failed to set speed slider");
        set_init_end_status(false, "speed slider set failed");
        return 2;
    }

    // Start IO Manager pub and srvs
    io_manager_->start(freq_rtde_hz_);

    // Advertise services
    ros::ServiceServer set_tcp_payload_srv = nh_->advertiseService<rpwc_msgs::setTcpPayload::RequestType, rpwc_msgs::setTcpPayload::ResponseType>("rpwc_set_tcp_payload", &callback_set_tcp_payload);
    ros::ServiceServer set_controller_srv = nh_->advertiseService<rpwc_msgs::setController::RequestType, rpwc_msgs::setController::ResponseType>("rpwc_controller", &callback_set_controller);
    ros::ServiceServer srv_get_controller = nh_->advertiseService<rpwc_msgs::getController::RequestType, rpwc_msgs::getController::ResponseType>("get_rpwc_controller", &callback_get_controller);
    ros::ServiceServer set_free_jog_params_srv = nh_->advertiseService<rpwc_msgs::setFreeJogParams::RequestType, rpwc_msgs::setFreeJogParams::ResponseType>("set_free_jog_params", &callback_set_free_jog_params);
    ros::ServiceServer get_free_jog_params_srv = nh_->advertiseService<rpwc_msgs::getFreeJogParams::RequestType, rpwc_msgs::getFreeJogParams::ResponseType>("get_free_jog_params", &callback_get_free_jog_params);
    ros::ServiceServer set_speed_override_srv = nh_->advertiseService<rpwc_msgs::setSpeedOverride::RequestType, rpwc_msgs::setSpeedOverride::ResponseType>("set_speed_override", &callback_set_speed_override);
    ros::ServiceServer get_speed_override_srv = nh_->advertiseService<rpwc_msgs::getSpeedOverride::RequestType, rpwc_msgs::getSpeedOverride::ResponseType>("get_speed_override", &callback_get_speed_override);

    CartesianMove cart_act_srv("native_cartesian_commands");
    JointsMove joint_act_srv("native_joints_commands");

    ROS_INFO("start");
    keep_alive_thread_ = std::thread(&thread_keep_alive);
    if (!set_init_end_status(true, "Node started"))
    {
        const std::string rtde_error = pending_rtde_failure();
        set_init_end_status(false, rtde_error.empty() ? "RTDE reader failed during startup" : rtde_error);
        return 1;
    }
    ros::waitForShutdown();

    ROS_INFO("Exiting");
    shutdown("Node shutdown");
    return 0;
}
