#include <ur_robot_driver/rpwc_bridge_native.hpp>

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

        // Wrench
        if (enable_wrench_publisher_ && wrench_mutex_.try_lock()) {
            data_pkg->getData<urcl::vector6d_t>("ft_raw_wrench", ft_raw_wrench_vec_);
            wrench_mutex_.unlock();
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

void thread_pub_wrench()
{
    ROS_INFO("[pub_wrench]: Init");
    ros::Publisher pub = nh_->advertise<geometry_msgs::Wrench>("wrench", 10, false);
    ros::Rate rate {freq_rtde_hz_};
    geometry_msgs::Wrench msg;
    urcl::vector6d_t local_wrench_vec;

    ROS_INFO("[pub_wrench]: Start");
    while (ros::ok())
    {
        {
            std::lock_guard<std::mutex> lk {wrench_mutex_};
            local_wrench_vec = ft_raw_wrench_vec_;
        }

        msg.force.x = local_wrench_vec[0];
        msg.force.y = local_wrench_vec[1];
        msg.force.z = local_wrench_vec[2];
        msg.torque.x = local_wrench_vec[3];
        msg.torque.y = local_wrench_vec[4];
        msg.torque.z = local_wrench_vec[5];

        pub.publish(msg);

        rate.sleep();
    }

    pub.shutdown();
    ROS_INFO("[pub_wrench]: End");
    return;
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

bool exec_traj(std::vector<std::shared_ptr<urcl::control::MotionPrimitive>> waypoints)
{
    if (!robot_state_manager_ || !robot_state_manager_->isReady())
        return false;

    std::lock_guard<std::mutex> send_command_lock_guard(send_command_mutex_);
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
        rpwc_result.success = false;
        rpwc_result.error_code = 9;
        rpwc_result.msg = "Node shutdown";
        as.setAborted(rpwc_result, rpwc_result.msg);
    }
    as.shutdown();
}

void CartesianMove::goal_callback()
{
    ROS_INFO("[Cartesian Move]: Accepting new goal");
    rpwc_goal = as.acceptNewGoal();

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

        if (!robot_state_manager_->isReady())
        {
            ROS_WARN("[Cartesian Move]: Robot program not ready, aborting goal");
            rpwc_result.success = false;
            rpwc_result.error_code = 9;
            rpwc_result.msg = "Robot program not ready";
            as.setAborted(rpwc_result, rpwc_result.msg);
            return;
        }
    }

    long unsigned int len = rpwc_goal->Poses.size();
    if (rpwc_goal->types.size() < len)
        len = rpwc_goal->types.size();
    if (rpwc_goal->velocities.size() < len)
        len = rpwc_goal->velocities.size();
    if (rpwc_goal->accelerations.size() < len)
        len = rpwc_goal->accelerations.size();
    if (rpwc_goal->zone_radiuses.size() < len)
        len = rpwc_goal->zone_radiuses.size();

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
            as.setAborted(rpwc_result, "Failed to confirm cancellation during safeguard stop");
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
        rpwc_result.success = false;
        rpwc_result.error_code = 9;
        rpwc_result.msg = "Node shutdown";
        as.setAborted(rpwc_result, rpwc_result.msg);
    }
    as.shutdown();
}

void JointsMove::goal_callback()
{
    ROS_INFO("[Joints Move]: Accepting new goal");
    rpwc_goal = as.acceptNewGoal();

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

        if (!robot_state_manager_->isReady())
        {
            ROS_WARN("[Joints Move]: Robot program not ready, aborting goal");
            rpwc_result.success = false;
            rpwc_result.error_code = 9;
            rpwc_result.msg = "Robot program not ready";
            as.setAborted(rpwc_result, rpwc_result.msg);
            return;
        }
    }

    long unsigned int len = rpwc_goal->targets.size();
    if (rpwc_goal->velocities.size() < len)
        len = rpwc_goal->velocities.size();
    if (rpwc_goal->accelerations.size() < len)
        len = rpwc_goal->accelerations.size();
    if (rpwc_goal->zone_radiuses.size() < len)
        len = rpwc_goal->zone_radiuses.size();

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
            as.setAborted(rpwc_result, "Failed to confirm cancellation during safeguard stop");
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

    bool auto_recover_protective_stop;
    double recovery_timeout_s;
    int recovery_retries;
    nh_->param<bool>("auto_recover_protective_stop", auto_recover_protective_stop, false);
    nh_->param<double>("recovery_timeout_s", recovery_timeout_s, 3.0);
    nh_->param<int>("recovery_retries", recovery_retries, 3);

    if (!nh_->getParam("enable_wrench_publisher", enable_wrench_publisher_))
    {
        ROS_ERROR_STREAM("Param '" << name_space_ << "/enable_wrench_publisher' not found on param server");
        set_init_end_status(false, "Param enable_wrench_publisher missing");
        return 1;
    }

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
        if (ur_instruction_executor_)
            ur_instruction_executor_->cancelMotion();
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

    if (enable_wrench_publisher_)
        std::thread wrench_pub_thread{&thread_pub_wrench};

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
