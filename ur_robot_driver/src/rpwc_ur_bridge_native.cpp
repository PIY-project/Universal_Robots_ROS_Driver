#include <ur_robot_driver/rpwc_bridge_native.hpp>
#include <algorithm>
#include <cmath>

// -----------------------------------------
//                Functions
// -----------------------------------------

void set_init_end_status(const bool success, const std::string &msg)
{
    init_status_ = 1;
    init_success_ = success;
    init_msg_ = msg;

    if (success)
        return;

    ros::waitForShutdown();
    shutdown(msg);
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
    ROS_INFO("[handle_rtde]: Init");
    ros::Rate rate(freq_rtde_hz_);
    std::unique_ptr<urcl::rtde_interface::DataPackage> data_pkg{new urcl::rtde_interface::DataPackage(ur_driver_->getRTDEOutputRecipe())};

    ROS_INFO("[handle_rtde]: Start");
    ur_driver_->startRTDECommunication(false);

    while (ros::ok())
    {
        if (!ur_driver_->getDataPackageBlocking(data_pkg))
        {
            rate.sleep();
            continue;
        }

        // Robot joints data
        {
            urcl::vector6d_t joints, vels;
            data_pkg->getData<urcl::vector6d_t>("actual_q", joints);
            data_pkg->getData<urcl::vector6d_t>("actual_qd", vels);
            kinematics_manager_->updateRtdeJointData(joints, vels);
        }

        // IO signals data
        {
            std::uint64_t in_bits, out_bits;
            data_pkg->getData<std::uint64_t>("actual_digital_input_bits", in_bits);
            data_pkg->getData<std::uint64_t>("actual_digital_output_bits", out_bits);
            ati_stop_monitor_.updateInputs(in_bits);
            io_manager_->digital_input_bits.store(in_bits, std::memory_order_relaxed);
            io_manager_->digital_output_bits.store(out_bits, std::memory_order_relaxed);
        }

        // Runtime state
        {
            uint32_t rt_state = 0;
            int32_t robot_mode = 0, safety_mode = 0;
            data_pkg->getData<uint32_t>("runtime_state", rt_state);
            data_pkg->getData<int32_t>("robot_mode", robot_mode);
            data_pkg->getData<int32_t>("safety_mode", safety_mode);
            robot_state_manager_->updateRtdeState(rt_state, robot_mode, safety_mode);
        }

        rate.sleep();
    }

    ROS_INFO("[handle_rtde]: Shutdown");
}

void thread_keep_alive()
{
    ros::Rate rate(freq_rtde_hz_);
    while (ros::ok())
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
    if (robot_state_manager_)
        robot_state_manager_->onProgramStateChanged(program_running);
}

void shutdown(std::string reason)
{
    ROS_WARN_STREAM("Shutting down node, reason: " << reason);
    if (nh_ != nullptr)
    {
        motor_off_on_shutdown_ = nh_->param("motor_off_for_deactivation", true);
        nh_->shutdown();
    }

    if (ur_primary_)
    {
        ur_primary_->commandStop();
        ur_primary_->stop();
    }

    if (ur_dashboard_)
    {
        ur_dashboard_->commandClearOperationalMode();
        if (motor_off_on_shutdown_)
            ur_dashboard_->commandPowerOff();
        ur_dashboard_->disconnect();
    }

    if (ur_driver_)
        ur_driver_->stopControl();

    ur_driver::unregisterUrclLogHandler();
    urcl::setLogLevel(urcl::LogLevel::INFO);
}

bool cancel_current_motion()
{
    std::lock_guard<std::mutex> lock(cancel_motion_mutex_);
    if (!ur_instruction_executor_)
        return false;
    try
    {
        return ur_instruction_executor_->cancelMotion();
    }
    catch (const std::exception &e)
    {
        ROS_ERROR_STREAM("[Motion]: Cancellation failed: " << e.what());
        return false;
    }
}

MotionResult exec_traj(std::vector<std::shared_ptr<urcl::control::MotionPrimitive>> waypoints)
{
    if (!robot_state_manager_ || !robot_state_manager_->isReady())
        return MotionResult::NOT_READY;

    std::lock_guard<std::mutex> send_command_lock_guard(send_command_mutex_);
    if (!robot_state_manager_->isReady())
        return MotionResult::NOT_READY;

    const auto goal = ati_stop_monitor_.startGoal();
    const std::chrono::duration<double> high_timeout(ati_high_goal_timeout_s_);
    std::atomic<bool> finished{false};
    std::atomic<bool> ati_stopped{false};
    std::thread watchdog([&]() {
        while (!finished.load(std::memory_order_acquire) && ros::ok())
        {
            if (ati_stop_monitor_.shouldStop(goal, high_timeout))
            {
                ati_stopped.store(true, std::memory_order_release);
                ROS_WARN("[ATI]: Stop condition detected during trajectory execution");
                if (!finished.load(std::memory_order_acquire))
                {
                    // The controller handles the rising edge itself. Cancellation
                    // also releases executeMotion() if its socket is still alive.
                    const bool canceled = cancel_current_motion();
                    if (!canceled && robot_state_manager_->isReady())
                    {
                        ROS_WARN("[ATI]: Motion cancellation unconfirmed; stopping robot program");
                        if (!robot_state_manager_->forceProgramStop())
                            ROS_ERROR("[ATI]: Dashboard program stop failed");
                    }
                }
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });

    bool success = false;
    try
    {
        success = ur_instruction_executor_->executeMotion(waypoints);
    }
    catch (const std::exception &e)
    {
        ROS_ERROR_STREAM("[Motion]: executeMotion failed: " << e.what());
    }
    const auto finished_at = AtiStopMonitor::Clock::now();
    const bool ati_at_finish = ati_stop_monitor_.shouldStop(goal, high_timeout, finished_at);
    finished.store(true, std::memory_order_release);
    watchdog.join();

    if (ati_stopped.load(std::memory_order_acquire) || ati_at_finish)
        return MotionResult::ATI_STOPPED;
    return success ? MotionResult::SUCCESS : MotionResult::FAILED;
}

MotionResult move_l(std::vector<geometry_msgs::Pose> waypoints, std::vector<double> velocities, std::vector<double> accelerations, std::vector<double> blending_radiuses)
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

MotionResult move_j(std::vector<KDL::JntArray> waypoints, std::vector<double> velocities, std::vector<double> accelerations, std::vector<double> blending_radiuses)
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
    res.status.data = init_status_;
    res.result.data = init_success_;
    res.info.data = init_msg_;
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
        if (req.controller != 99)
            last_controller_started_ = req.controller;
        else
            last_controller_started_ = 0;

        std::lock_guard<std::mutex> lock(send_command_mutex_);
        res.result.data = ur_driver_->writeFreedriveControlMessage(urcl::control::FreedriveControlMessage::FREEDRIVE_STOP);
        freedrive_ = false;
        return true;
    }

    ROS_INFO("Start freedrive");
    last_controller_started_ = 2;
    std::lock_guard<std::mutex> lock(send_command_mutex_);
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

template <typename ActionServer>
bool wait_for_program_ready(ActionServer &server)
{
    const auto deadline = AtiStopMonitor::Clock::now() +
                          std::chrono::duration<double>(program_ready_wait_s_);
    while (ros::ok() && server.isActive() && robot_state_manager_ &&
           !robot_state_manager_->isReady() && !server.isPreemptRequested())
    {
        if (!robot_state_manager_->isSafeguardActive() && AtiStopMonitor::Clock::now() >= deadline)
            break;
        ros::Duration(0.1).sleep();
    }
    return server.isActive() && !server.isPreemptRequested() && robot_state_manager_ &&
           robot_state_manager_->isReady();
}

template <typename Result>
void set_motion_result(Result &result, MotionResult motion)
{
    result.success = motion == MotionResult::SUCCESS;
    if (motion == MotionResult::ATI_STOPPED)
    {
        result.error_code = static_cast<int16_t>(rpwc::errorArmMotionCode::STOP_ATI_SENSOR);
        result.msg = "Motion stopped by ATI sensor (DI0)";
    }
    else if (motion == MotionResult::NOT_READY)
    {
        result.error_code = static_cast<int16_t>(rpwc::errorArmMotionCode::GENERIC_ERROR);
        result.msg = "External Control unavailable";
    }
    else if (motion == MotionResult::FAILED)
    {
        result.error_code = static_cast<int16_t>(rpwc::errorArmMotionCode::GENERIC_ERROR);
        result.msg = "Trajectory execution failed";
    }
    else
    {
        result.error_code = static_cast<int16_t>(rpwc::errorArmMotionCode::NO_ERROR);
        result.msg = "Goal succeeded";
    }
}

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
        result.error_code = static_cast<int16_t>(rpwc::errorArmMotionCode::GENERIC_ERROR);
        result.msg = "Node shutdown";
        as.setAborted(result, result.msg);
    }
    as.shutdown();
}

void CartesianMove::goal_callback()
{
    ROS_INFO("[Cartesian Move]: Accepting new goal");
    const auto epoch = goal_epoch_.fetch_add(1, std::memory_order_acq_rel) + 1;
    const auto goal = as.acceptNewGoal();
    rpwc_msgs::nativeCartesianCommandsResult result;

    if (!wait_for_program_ready(as))
    {
        if (!as.isActive() || epoch != goal_epoch_.load(std::memory_order_acquire))
            return;
        if (as.isPreemptRequested())
        {
            result.success = false;
            result.error_code = static_cast<int16_t>(rpwc::errorArmMotionCode::STOP_RUN);
            result.msg = "Goal canceled";
            as.setPreempted(result, result.msg);
            return;
        }
        ROS_WARN("[Cartesian Move]: External Control unavailable, aborting goal");
        set_motion_result(result, MotionResult::NOT_READY);
        as.setAborted(result, result.msg);
        return;
    }
    if (epoch != goal_epoch_.load(std::memory_order_acquire))
        return;

    const auto len = goal->Poses.size();

    ROS_INFO_STREAM("[Cartesian Move]: Max valid poses: " << len);
    if (len == 0 || goal->types.size() != len || goal->velocities.size() != len ||
        goal->accelerations.size() != len || goal->zone_radiuses.size() != len)
    {
        ROS_WARN("[Cartesian Move]: Invalid trajectory data, aborting");
        result.success = false;
        result.error_code = static_cast<int16_t>(rpwc::errorArmMotionCode::GENERIC_ERROR);
        result.msg = "Invalid trajectory data";
        as.setAborted(result, result.msg);
        return;
    }

    for (const auto &type : goal->types)
    {
        if (type != goal->LINEAR_MOVE)
        {
            ROS_ERROR_STREAM("[Cartesian Move]: Uknown move type: '" << type << "' aborting goal");
            result.success = false;
            result.error_code = static_cast<int16_t>(rpwc::errorArmMotionCode::GENERIC_ERROR);
            result.msg = "Unknown move type";
            as.setAborted(result, result.msg);
            return;
        }
    }

    ROS_INFO("[Cartesian Move]: Executing trajectory");
    const MotionResult motion = move_l(goal->Poses, goal->velocities, goal->accelerations, goal->zone_radiuses);
    if (!as.isActive() || epoch != goal_epoch_.load(std::memory_order_acquire))
        return;
    if (as.isPreemptRequested())
    {
        result.success = false;
        result.error_code = static_cast<int16_t>(rpwc::errorArmMotionCode::STOP_RUN);
        result.msg = "Goal canceled";
        as.setPreempted(result, result.msg);
        return;
    }
    set_motion_result(result, motion);

    if (!result.success)
    {
        ROS_WARN_STREAM("[Cartesian Move]: Goal aborted: " << result.msg);
        as.setAborted(result, result.msg);
        return;
    }

    ROS_INFO("[Cartesian Move]: Goal completed");
    as.setSucceeded(result, result.msg);
}

void CartesianMove::preempt_callback()
{
    ROS_INFO("[Cartesian Move]: Goal preempted");
    const auto epoch = goal_epoch_.load(std::memory_order_acquire);
    if (!as.isActive())
        return;

    rpwc_msgs::nativeCartesianCommandsResult result;
    result.success = false;
    result.error_code = static_cast<int16_t>(rpwc::errorArmMotionCode::STOP_RUN);
    result.msg = "Goal canceled";

    if (robot_state_manager_ && robot_state_manager_->isSafeguardActive())
    {
        ROS_WARN("[Cartesian Move]: Safeguard active, forcing program stop to cancel motion");
        if (robot_state_manager_->forceProgramStop())
        {
            if (epoch == goal_epoch_.load(std::memory_order_acquire) && as.isActive())
                as.setPreempted(result, result.msg);
        }
        else
        {
            ROS_ERROR("[Cartesian Move]: Failed to force-stop robot program, falling back to cancelMotion");
            cancel_current_motion();
            result.error_code = static_cast<int16_t>(rpwc::errorArmMotionCode::GENERIC_ERROR);
            result.msg = "Failed to confirm cancellation during safeguard stop";
            if (epoch == goal_epoch_.load(std::memory_order_acquire) && as.isActive())
                as.setAborted(result, result.msg);
        }
    }
    else
    {
        cancel_current_motion();
        if (epoch == goal_epoch_.load(std::memory_order_acquire) && as.isActive())
            as.setPreempted(result, result.msg);
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
        result.error_code = static_cast<int16_t>(rpwc::errorArmMotionCode::GENERIC_ERROR);
        result.msg = "Node shutdown";
        as.setAborted(result, result.msg);
    }
    as.shutdown();
}

void JointsMove::goal_callback()
{
    ROS_INFO("[Joints Move]: Accepting new goal");
    const auto epoch = goal_epoch_.fetch_add(1, std::memory_order_acq_rel) + 1;
    const auto goal = as.acceptNewGoal();
    rpwc_msgs::nativeJointsCommandsResult result;

    if (!wait_for_program_ready(as))
    {
        if (!as.isActive() || epoch != goal_epoch_.load(std::memory_order_acquire))
            return;
        if (as.isPreemptRequested())
        {
            result.success = false;
            result.error_code = static_cast<int16_t>(rpwc::errorArmMotionCode::STOP_RUN);
            result.msg = "Goal canceled";
            as.setPreempted(result, result.msg);
            return;
        }
        ROS_WARN("[Joints Move]: External Control unavailable, aborting goal");
        set_motion_result(result, MotionResult::NOT_READY);
        as.setAborted(result, result.msg);
        return;
    }
    if (epoch != goal_epoch_.load(std::memory_order_acquire))
        return;

    const auto len = goal->targets.size();

    ROS_INFO_STREAM("[Joints Move]: Max valid poses: " << len);
    if (len == 0 || goal->velocities.size() != len || goal->accelerations.size() != len ||
        goal->zone_radiuses.size() != len)
    {
        ROS_WARN("[Joints Move]: Invalid trajectory data, aborting");
        result.success = false;
        result.error_code = static_cast<int16_t>(rpwc::errorArmMotionCode::GENERIC_ERROR);
        result.msg = "Invalid trajectory data";
        as.setAborted(result, result.msg);
        return;
    }

    std::vector<KDL::JntArray> waypoints;
    KDL::JntArray tmpWaypoint;
    tmpWaypoint.resize(kinematics_manager_->getNumJoints());
    for (const auto &target : goal->targets)
    {
        if (target.values.size() != static_cast<std::size_t>(kinematics_manager_->getNumJoints()))
        {
            result.success = false;
            result.error_code = static_cast<int16_t>(rpwc::errorArmMotionCode::GENERIC_ERROR);
            result.msg = "Invalid joint target size";
            as.setAborted(result, result.msg);
            return;
        }
        for (int i = 0; i < kinematics_manager_->getNumJoints(); i++)
        {
            tmpWaypoint(i) = target.values[i];
        }
        waypoints.push_back(tmpWaypoint);
    }

    ROS_INFO("[Joints Move]: Executing trajectory");
    const MotionResult motion = move_j(waypoints, goal->velocities, goal->accelerations, goal->zone_radiuses);
    if (!as.isActive() || epoch != goal_epoch_.load(std::memory_order_acquire))
        return;
    if (as.isPreemptRequested())
    {
        result.success = false;
        result.error_code = static_cast<int16_t>(rpwc::errorArmMotionCode::STOP_RUN);
        result.msg = "Goal canceled";
        as.setPreempted(result, result.msg);
        return;
    }
    set_motion_result(result, motion);

    if (!result.success)
    {
        ROS_WARN_STREAM("[Joints Move]: Goal aborted: " << result.msg);
        as.setAborted(result, result.msg);
        return;
    }

    ROS_INFO("[Joints Move]: Goal completed");
    as.setSucceeded(result, result.msg);
}

void JointsMove::preempt_callback()
{
    ROS_INFO("[Joints Move]: Goal preempted");
    const auto epoch = goal_epoch_.load(std::memory_order_acquire);
    if (!as.isActive())
        return;

    rpwc_msgs::nativeJointsCommandsResult result;
    result.success = false;
    result.error_code = static_cast<int16_t>(rpwc::errorArmMotionCode::STOP_RUN);
    result.msg = "Goal canceled";

    if (robot_state_manager_ && robot_state_manager_->isSafeguardActive())
    {
        ROS_WARN("[Joints Move]: Safeguard active, forcing program stop to cancel motion");
        if (robot_state_manager_->forceProgramStop())
        {
            if (epoch == goal_epoch_.load(std::memory_order_acquire) && as.isActive())
                as.setPreempted(result, result.msg);
        }
        else
        {
            ROS_ERROR("[Joints Move]: Failed to force-stop robot program, falling back to cancelMotion");
            cancel_current_motion();
            result.error_code = static_cast<int16_t>(rpwc::errorArmMotionCode::GENERIC_ERROR);
            result.msg = "Failed to confirm cancellation during safeguard stop";
            if (epoch == goal_epoch_.load(std::memory_order_acquire) && as.isActive())
                as.setAborted(result, result.msg);
        }
    }
    else
    {
        cancel_current_motion();
        if (epoch == goal_epoch_.load(std::memory_order_acquire) && as.isActive())
            as.setPreempted(result, result.msg);
    }
}

// -----------------------------------------
//                  Main
// -----------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rpwc_ur_bridge_native");
    nh_ = new ros::NodeHandle();
    ros::AsyncSpinner spinner(4);
    spinner.start();

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
    init_status_ = 0;
    init_success_ = false;
    init_msg_ = "";

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
    nh_->param<double>("ati_high_goal_timeout_s", ati_high_goal_timeout_s_, 2.0);
    if (!std::isfinite(ati_high_goal_timeout_s_) || ati_high_goal_timeout_s_ <= 0.0)
    {
        ROS_FATAL("ati_high_goal_timeout_s must be a positive finite number");
        ros::requestShutdown();
        set_init_end_status(false, "Invalid ati_high_goal_timeout_s");
        return 1;
    }

    bool auto_recover_protective_stop;
    double recovery_timeout_s;
    int recovery_retries;
    nh_->param<bool>("auto_recover_protective_stop", auto_recover_protective_stop, false);
    nh_->param<double>("recovery_timeout_s", recovery_timeout_s, 3.0);
    nh_->param<int>("recovery_retries", recovery_retries, 3);
    program_ready_wait_s_ = std::max(2.0, recovery_timeout_s * std::max(1, recovery_retries) + 2.0);

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

    ur_driver_.reset(new urcl::UrDriver(urDriverConfig));
    ur_driver_->resetRTDEClient(urDriverConfig.output_recipe_file, urDriverConfig.input_recipe_file, freq_rtde_hz_, true);
    ROS_INFO_STREAM("ControlFrequency: " << ur_driver_->getControlFrequency());

    // Init IO Manager
    std::string io_json_path;
    ros::param::get("io_signals_names_path", io_json_path);
    io_manager_.reset(new IOManager(*nh_, ur_driver_, io_json_path));

    // Init Robot State Manager
    robot_state_manager_.reset(new RobotStateManager(*nh_, ur_driver_, ur_dashboard_, auto_recover_protective_stop, recovery_timeout_s, recovery_retries));
    robot_state_manager_->setOnBlockedCallback([]() {
        cancel_current_motion();
    });
    robot_state_manager_->setOnProgramRestartedCallback([]() {
        if (!kinematics_manager_)
            return;
        try
        {
            kinematics_manager_->applyRobotConfig();
        }
        catch (const std::exception &e)
        {
            ROS_ERROR_STREAM("[ProgramRestarted]: Failed to reapply TCP/payload config: " << e.what());
        }
    });

    std::thread rtde_thread{&thread_handle_rtde};
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

    // Init Kinematics Manager (parses URDF, builds KDL chains, sets TCP offset)
    try
    {
        kinematics_manager_.reset(new KinematicsManager(*nh_, ur_driver_));
    }
    catch (const std::exception &e)
    {
        set_init_end_status(false, e.what());
        return 1;
    }
    // Wait for program start — state manager handles recovery and shutdown on failure
    while (!robot_state_manager_->isReady() && ros::ok())
        ros::Duration(0.05).sleep();

    if (!ros::ok())
    {
        set_init_end_status(false, "Program start failed");
        return 1;
    }

    // Apply robot configuration (TCP offset + payload) and start KM threads
    try
    {
        kinematics_manager_->applyRobotConfig();
    }
    catch (const std::exception &e)
    {
        set_init_end_status(false, e.what());
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
    ros::ServiceServer set_controller_srv = nh_->advertiseService<rpwc_msgs::setController::RequestType, rpwc_msgs::setController::ResponseType>("rpwc_controller", &callback_set_controller);
    ros::ServiceServer srv_get_controller = nh_->advertiseService<rpwc_msgs::getController::RequestType, rpwc_msgs::getController::ResponseType>("get_rpwc_controller", &callback_get_controller);
    ros::ServiceServer set_free_jog_params_srv = nh_->advertiseService<rpwc_msgs::setFreeJogParams::RequestType, rpwc_msgs::setFreeJogParams::ResponseType>("set_free_jog_params", &callback_set_free_jog_params);
    ros::ServiceServer get_free_jog_params_srv = nh_->advertiseService<rpwc_msgs::getFreeJogParams::RequestType, rpwc_msgs::getFreeJogParams::ResponseType>("get_free_jog_params", &callback_get_free_jog_params);
    ros::ServiceServer set_speed_override_srv = nh_->advertiseService<rpwc_msgs::setSpeedOverride::RequestType, rpwc_msgs::setSpeedOverride::ResponseType>("set_speed_override", &callback_set_speed_override);
    ros::ServiceServer get_speed_override_srv = nh_->advertiseService<rpwc_msgs::getSpeedOverride::RequestType, rpwc_msgs::getSpeedOverride::ResponseType>("get_speed_override", &callback_get_speed_override);

    CartesianMove cart_act_srv("native_cartesian_commands");
    JointsMove joint_act_srv("native_joints_commands");

    ROS_INFO("start");
    std::thread keep_alive(&thread_keep_alive);
    set_init_end_status(true, "Node started");
    ros::waitForShutdown();

    ROS_INFO("Exiting");
    spinner.stop();
    shutdown("Node shutdown");
    return 0;
}
