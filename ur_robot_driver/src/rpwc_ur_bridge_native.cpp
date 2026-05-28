#include <ur_robot_driver/rpwc_bridge_native.hpp>

namespace
{
    constexpr size_t kStandardDigitalSignalCount = 8;
    constexpr size_t kConfigurableDigitalSignalCount = 8;
    constexpr size_t kToolDigitalSignalCount = 2;
    constexpr size_t kTotalDigitalSignalCount = kStandardDigitalSignalCount + kConfigurableDigitalSignalCount + kToolDigitalSignalCount;

    std::array<std::string, kTotalDigitalSignalCount> digital_signal_names_{};
    std::unordered_map<std::string, size_t> digital_signal_name_to_bit_{};

    std::string getDefaultDigitalSignalName(const size_t bit)
    {
        if (bit < kStandardDigitalSignalCount)
            return "standard_" + std::to_string(bit);

        if (bit < kStandardDigitalSignalCount + kConfigurableDigitalSignalCount)
            return "configurable_" + std::to_string(bit - kStandardDigitalSignalCount);

        return "tool_" + std::to_string(bit - kStandardDigitalSignalCount - kConfigurableDigitalSignalCount);
    }

    void setDefaultDigitalSignalNames()
    {
        for (size_t bit = 0; bit < digital_signal_names_.size(); ++bit)
            digital_signal_names_[bit] = getDefaultDigitalSignalName(bit);
    }

    bool rebuildDigitalSignalLookup(std::string &error_message)
    {
        digital_signal_name_to_bit_.clear();

        for (size_t bit = 0; bit < digital_signal_names_.size(); ++bit)
        {
            const std::string &signal_name = digital_signal_names_[bit];

            if (signal_name.empty())
            {
                error_message = "Resolved digital signal name is empty for bit " + std::to_string(bit);
                return false;
            }

            if (!digital_signal_name_to_bit_.emplace(signal_name, bit).second)
            {
                error_message = "Duplicate digital signal name: " + signal_name;
                return false;
            }
        }

        return true;
    }

    bool loadDigitalSignalGroup(const boost::property_tree::ptree &digital_inputs, const std::string &group_name, const size_t expected_size, const size_t bit_offset, std::string &error_message)
    {
        const auto group_tree = digital_inputs.get_child_optional(group_name);
        if (!group_tree)
        {
            error_message = "Missing digital_inputs." + group_name + " array in io signal names JSON";
            return false;
        }

        size_t index = 0;
        for (const auto &entry : *group_tree)
        {
            if (index >= expected_size)
            {
                error_message = "digital_inputs." + group_name + " must contain exactly " + std::to_string(expected_size) + " entries";
                return false;
            }

            const std::string configured_name = entry.second.get_value<std::string>();
            if (!configured_name.empty())
                digital_signal_names_[bit_offset + index] = configured_name;

            ++index;
        }

        if (index != expected_size)
        {
            error_message = "digital_inputs." + group_name + " must contain exactly " + std::to_string(expected_size) + " entries";
            return false;
        }

        return true;
    }

    bool loadDigitalSignalNamesFromJson(const std::string &json_path, std::string &error_message)
    {
        std::ifstream json_file(json_path);
        if (!json_file.is_open())
            return false;

        boost::property_tree::ptree root;
        try
        {
            boost::property_tree::read_json(json_file, root);
        }
        catch (const boost::property_tree::json_parser::json_parser_error &err)
        {
            error_message = err.message() + " at line " + std::to_string(err.line());
            return false;
        }

        const auto digital_inputs = root.get_child_optional("digital_inputs");
        if (!digital_inputs)
        {
            error_message = "Missing digital_inputs object in io signal names JSON";
            return false;
        }

        try
        {
            setDefaultDigitalSignalNames();

            if (!loadDigitalSignalGroup(*digital_inputs, "standard", kStandardDigitalSignalCount, 0, error_message))
                return false;

            if (!loadDigitalSignalGroup(*digital_inputs, "configurable", kConfigurableDigitalSignalCount, kStandardDigitalSignalCount, error_message))
                return false;

            if (!loadDigitalSignalGroup(*digital_inputs, "tool", kToolDigitalSignalCount, kStandardDigitalSignalCount + kConfigurableDigitalSignalCount, error_message))
                return false;
        }
        catch (const boost::property_tree::ptree_error &err)
        {
            error_message = err.what();
            return false;
        }

        return rebuildDigitalSignalLookup(error_message);
    }

    bool resolveDigitalSignalBit(const std::string &signal_name, size_t &bit)
    {
        const auto it = digital_signal_name_to_bit_.find(signal_name);
        if (it == digital_signal_name_to_bit_.end())
            return false;

        bit = it->second;
        return true;
    }

    bool setResolvedDigitalOutput(const size_t bit, const bool value)
    {
        if (bit < kStandardDigitalSignalCount)
            return ur_driver_->getRTDEWriter().sendStandardDigitalOutput(static_cast<uint8_t>(bit), value);

        if (bit < kStandardDigitalSignalCount + kConfigurableDigitalSignalCount)
            return ur_driver_->getRTDEWriter().sendConfigurableDigitalOutput(static_cast<uint8_t>(bit - kStandardDigitalSignalCount), value);

        return ur_driver_->getRTDEWriter().sendToolDigitalOutput(static_cast<uint8_t>(bit - kStandardDigitalSignalCount - kConfigurableDigitalSignalCount), value);
    }
} // namespace

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

void thread_keep_alive()
{
    ros::Rate rate(freq_rtde_hz_);
    while (ros::ok())
    {
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

void thread_handle_rtde()
{
    ROS_INFO("[handle_rtde]: Init");
    ros::Rate rate(freq_rtde_hz_);
    std::unique_ptr<urcl::rtde_interface::DataPackage> data_pkg{new urcl::rtde_interface::DataPackage(ur_driver_->getRTDEOutputRecipe())};

    joint_data_mutex_.lock();
    rob_joints_ = {};
    rob_joints_vel_ = {};
    joint_data_mutex_.unlock();

    io_signals_data_mutex_.lock();
    rob_io_signals_ = 0x0;
    io_signals_data_mutex_.unlock();

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
            std::lock_guard lk{joint_data_mutex_};
            data_pkg->getData<urcl::vector6d_t>("actual_q", rob_joints_);
            data_pkg->getData<urcl::vector6d_t>("actual_qd", rob_joints_vel_);
        }

        // IO signals data
        {
            std::lock_guard lk{io_signals_data_mutex_};
            data_pkg->getData<std::uint64_t>("actual_digital_input_bits", rob_io_signals_);
        }

        // Runtime state
        {
            uint32_t rt_state;
            data_pkg->getData<uint32_t>("runtime_state", rt_state);
            rtde_runtime_state_.store(rt_state, std::memory_order_relaxed);
        }

        rate.sleep();
    }

    ROS_INFO("[handle_rtde]: Shutdown");
}

void thread_pub_joint_states()
{
    ROS_INFO("[joint_states]: Init");

    sensor_msgs::JointState msg;
    msg.name.clear();
    if (!nh_->getParam("ur_hardware_interface/joints", msg.name))
    {
        ROS_FATAL_STREAM("[joint_states]: Parameter '" << name_space_ << "/ur_hardware_interface/joints' not found on param server");
        shutdown("Joint names not found on param server");
        return;
    }

    ros::Rate rate(freq_rtde_hz_);

    ros::Publisher pub = nh_->advertise<sensor_msgs::JointState>("joint_states", 1, false);

    ROS_INFO("[joint_states]: Start");

    while (ros::ok())
    {
        msg.position.clear();
        msg.velocity.clear();
        msg.effort.clear();
        msg.header.stamp = ros::Time::now();

        {
            std::lock_guard lk{joint_data_mutex_};

            for (int i = 0; i < num_of_joints_; i++)
            {
                q_msr_(i) = rob_joints_[i];
                msg.position.push_back(rob_joints_[i]);
                msg.effort.push_back(0.0);
            }

            for (double vel : rob_joints_vel_)
                msg.velocity.push_back(vel);
        }

        pub.publish(msg);
        rate.sleep();
    }

    ROS_INFO("[joint_states]: Shutting down");
    pub.shutdown();
}

void thread_pub_rob_curr_pose()
{
    ROS_INFO("[robot_curr_pose]: Init");

    Eigen::Vector3d pos_ee_msr, pos_ll_msr;
    Eigen::Quaterniond quat_ee_msr, quat_ll_msr;
    first_quat_ee_msr_ = true;
    first_quat_ll_msr_ = true;

    double dt_pub_pose;
    nh_->param<double>("dt_pub_pose", dt_pub_pose, 0.02);
    ros::Rate r_HZ(1.0 / dt_pub_pose);

    ros::Publisher pub = nh_->advertise<rpwc_msgs::RobotArmStateStamped>("rpwc_robot_curr_pose", 1);
    ros::ServiceServer server_robot_curr_pose = nh_->advertiseService("rpwc_robot_curr_pose", callback_robot_curr_pose);
    curr_pose_ee_.header.frame_id = root_name_;
    curr_pose_ll_.header.frame_id = root_name_;
    rpwc_msgs::RobotArmStateStamped msg;
    std_msgs::Float64 tmp;

    ROS_INFO("[robot_curr_pose]: Start");

    while (ros::ok())
    {
        fwdKin(fk_pos_solver_ee_, q_msr_, first_quat_ee_msr_, pos_ee_msr, quat_ee_msr, quat_ee_old_msr_);
        curr_pose_ee_.header.stamp = ros::Time::now();
        curr_pose_ee_.pose.position.x = pos_ee_msr.x();
        curr_pose_ee_.pose.position.y = pos_ee_msr.y();
        curr_pose_ee_.pose.position.z = pos_ee_msr.z();
        curr_pose_ee_.pose.orientation.w = quat_ee_msr.w();
        curr_pose_ee_.pose.orientation.x = quat_ee_msr.x();
        curr_pose_ee_.pose.orientation.y = quat_ee_msr.y();
        curr_pose_ee_.pose.orientation.z = quat_ee_msr.z();
        fwdKin(fk_pos_solver_ll_, q_msr_, first_quat_ll_msr_, pos_ll_msr, quat_ll_msr, quat_ll_old_msr_);
        curr_pose_ll_.header.stamp = ros::Time::now();
        curr_pose_ll_.pose.position.x = pos_ll_msr.x();
        curr_pose_ll_.pose.position.y = pos_ll_msr.y();
        curr_pose_ll_.pose.position.z = pos_ll_msr.z();
        curr_pose_ll_.pose.orientation.w = quat_ll_msr.w();
        curr_pose_ll_.pose.orientation.x = quat_ll_msr.x();
        curr_pose_ll_.pose.orientation.y = quat_ll_msr.y();
        curr_pose_ll_.pose.orientation.z = quat_ll_msr.z();

        msg.poseBaseToEe = curr_pose_ee_;
        msg.poseBaseToLastLink = curr_pose_ll_;
        msg.joint_position.clear();
        for (int i = 0; i < num_of_joints_; i++)
        {
            tmp.data = q_msr_(i);
            msg.joint_position.push_back(tmp);
        }
        pub.publish(msg);

        r_HZ.sleep();
    }

    ROS_INFO("[robot_curr_pose]: Shutting down");
    pub.shutdown();
    server_robot_curr_pose.shutdown();
}

void thread_pub_io_signals_state()
{
    ROS_INFO("[io_signals_state]: Init");

    ros::Rate rate{freq_rtde_hz_};

    ros::Publisher pub = nh_->advertise<rpwc_msgs::robotIOSignals>("io_signals_state", 1, true);
    std::uint64_t bits, old_bits = 0x0;
    bool first_publish = true;
    rpwc_msgs::robotIOSignals msg;
    rpwc_msgs::robotIOSignals::_digitalSignals_type::value_type digital_tmp;

    ROS_INFO("[io_signals_state]: Start");

    while (ros::ok())
    {
        {
            std::lock_guard lk{io_signals_data_mutex_};
            bits = rob_io_signals_;
        }

        if (!first_publish && bits == old_bits)
        {
            rate.sleep();
            continue;
        }

        first_publish = false;
        old_bits = bits;

        msg.digitalSignals.clear();
        for (size_t i = 0; i < kTotalDigitalSignalCount; ++i)
        {
            digital_tmp.signalName.data = digital_signal_names_[i];
            digital_tmp.value.data = static_cast<bool>((bits >> i) & 0x1);

            msg.digitalSignals.push_back(digital_tmp);
        }

        pub.publish(msg);
        rate.sleep();
    }

    ROS_INFO("[io_signals_state]: Shutting down");
}

void fwdKin(std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver, KDL::JntArray q, bool &first_quat, Eigen::Vector3d &pos, Eigen::Quaterniond &quat, Eigen::Quaterniond &quat_old)
{
    Eigen::Matrix3d orient;
    KDL::Frame x; // Tip pose
    fk_solver->JntToCart(q, x);

    for (int i = 0; i < 3; i++)
    {
        pos(i) = x.p(i);

        for (int j = 0; j < 3; j++)
        {
            orient(i, j) = x.M(i, j);
        }
    }

    // from matrix to quat
    quat = orient;
    quat.normalize();

    if (first_quat)
    {
        first_quat = false;
        quat_old = quat;
    }

    // rotation to quaternion issue , "Sign Flip" , check  http://www.dtic.mil/dtic/tr/fulltext/u2/1043624.pdf
    double sign_check = quat.w() * quat_old.w() + quat.x() * quat_old.x() + quat.y() * quat_old.y() + quat.z() * quat_old.z();
    if (sign_check < 0.0)
    {
        quat.w() = quat.w() * (-1);
        quat.vec() = quat.vec() * (-1);
    }

    quat_old = quat;
}

void shutdown(std::string reason)
{
    ROS_WARN_STREAM("Shutting down node, reason: " << reason);
    if (nh_ != nullptr)
        nh_->shutdown();

    if (ur_primary_)
    {
        ur_primary_->commandStop();
        ur_primary_->stop();
    }

    if (ur_dashboard_)
    {
        if (!motor_state_on_start_)
            ur_dashboard_->commandPowerOff();
        ur_dashboard_->commandClearOperationalMode();
        ur_dashboard_->disconnect();
    }

    if (ur_driver_)
        ur_driver_->stopControl();

    ur_driver::unregisterUrclLogHandler();
    urcl::setLogLevel(urcl::LogLevel::INFO);
}

void handleRobotProgramState(bool program_running)
{
    ROS_WARN_STREAM("ProgamState changed: " << (program_running ? "RUNNING" : "STOPPED"));

    std::string robot_mode;
    if (!ur_dashboard_->commandRobotMode(robot_mode))
    {
        ROS_ERROR("Failed to get robot mode");
        shutdown("Failed to get robot mode after program state change");
        return;
    }

    std::string safety_mode;
    if (!ur_dashboard_->commandSafetyMode(safety_mode))
    {
        ROS_ERROR("Failed to get safety mode");
        shutdown("Failed to get safety mode after program state change");
        return;
    }

    std::string safety_status;
    if (!ur_dashboard_->commandSafetyStatus(safety_status))
    {
        ROS_ERROR("Failed to get safety status");
        shutdown("Failed to get safety status after program state change");
        return;
    }

    ROS_INFO_STREAM("Robot Mode: " << robot_mode << " | Safety Mode: " << safety_mode << " | Safety Status: " << safety_status);
}

bool exec_traj(std::vector<std::shared_ptr<urcl::control::MotionPrimitive>> waypoints)
{
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
        for (int j = 0; j < num_of_joints_; j++)
        {
            joints[j] = waypoints[i](j);
        }
        vel = max_speed_joint_ * velocities[i];
        acc = max_acceleration_joint_ * accelerations[i];
        ROS_WARN_STREAM("vel: " << vel << " | acc: " << acc);

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

bool callback_robot_curr_pose(rpwc_msgs::robotArmState::Request &req, rpwc_msgs::robotArmState::Response &res)
{
    res.poseBaseToEe.header.stamp = ros::Time::now();
    res.poseBaseToEe.header.frame_id = curr_pose_ee_.header.frame_id;
    res.poseBaseToEe.pose = curr_pose_ee_.pose;
    res.poseBaseToLastLink.header.stamp = ros::Time::now();
    res.poseBaseToLastLink.header.frame_id = curr_pose_ll_.header.frame_id;
    res.poseBaseToLastLink.pose = curr_pose_ll_.pose;
    for (int i = 0; i < num_of_joints_; i++)
    {
        std_msgs::Float64 tmp;
        tmp.data = q_msr_(i);
        res.joint_position.push_back(tmp);
    }
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

bool callback_set_payload(rpwc_msgs::setPayload::Request &req, rpwc_msgs::setPayload::Response &res)
{
    if (req.mass.data < 0)
    {
        res.result.data = false;
        res.info.data = "Payload mass cannot be negative";
        return true;
    }
    if (req.center_of_mass.x == 0 && req.center_of_mass.y == 0 && req.center_of_mass.z == 0)
    {
        res.result.data = false;
        res.info.data = "Center of mass values must be different than zero";
        return true;
    }
    std::lock_guard<std::mutex> lock(send_command_mutex_);
    double payload = req.mass.data;

    urcl::vector3d_t cog_ur;
    KDL::Vector p_last(req.center_of_mass.x, req.center_of_mass.y, req.center_of_mass.z);
    KDL::Vector p_tool0 = t_tool02LastLink * p_last;
    cog_ur[0] = p_tool0.x();
    cog_ur[1] = p_tool0.y();
    cog_ur[2] = p_tool0.z();

    res.result.data = ur_driver_->setPayload(payload, cog_ur);
    return true;
}

bool callback_set_digital_io_signal(rpwc_msgs::setDigitalIOSignal::Request &req, rpwc_msgs::setDigitalIOSignal::Response &res)
{
    size_t bit = 0;
    if (!resolveDigitalSignalBit(req.signal_name, bit))
    {
        res.success = false;
        res.message = "Unknown digital IO signal name: " + req.signal_name;
        return true;
    }

    std::lock_guard<std::mutex> lock(send_command_mutex_);
    res.success = setResolvedDigitalOutput(bit, req.value);
    if (!res.success)
        res.message = "Failed to set digital IO signal: " + req.signal_name;

    return true;
}

bool callback_get_digital_io_signal(rpwc_msgs::getDigitalIOSignal::Request &req, rpwc_msgs::getDigitalIOSignal::Response &res)
{
    size_t bit = 0;
    if (!resolveDigitalSignalBit(req.signal_name, bit))
    {
        res.success = false;
        res.message = "Unknown digital IO signal name: " + req.signal_name;
        res.value = false;
        return true;
    }

    std::uint64_t bits = 0x0;
    {
        std::lock_guard lk{io_signals_data_mutex_};
        bits = rob_io_signals_;
    }

    res.value = static_cast<bool>((bits >> bit) & 0x1);
    res.success = true;
    res.message.clear();
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
        rpwc_result.msg = "Node shutdown";
        as.setAborted(rpwc_result, rpwc_result.msg);
    }
    as.shutdown();
}

void CartesianMove::goal_callback()
{
    ROS_INFO("[Cartesian Move]: Accepting new goal");
    rpwc_goal = as.acceptNewGoal();

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
    as.setPreempted();
    ur_instruction_executor_->cancelMotion();
    send_command_mutex_.unlock();
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
        rpwc_result.msg = "Node shutdown";
        as.setAborted(rpwc_result, rpwc_result.msg);
    }
    as.shutdown();
}

void JointsMove::goal_callback()
{
    ROS_INFO("[Joints Move]: Accepting new goal");
    rpwc_goal = as.acceptNewGoal();

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
        rpwc_result.msg = "No data found";
        as.setAborted(rpwc_result, rpwc_result.msg);
        return;
    }

    std::vector<KDL::JntArray> waypoints;
    KDL::JntArray tmpWaypoint;
    tmpWaypoint.resize(num_of_joints_);
    for (auto it = rpwc_goal->targets.begin(); it != rpwc_goal->targets.end(); it++)
    {
        for (int i = 0; i < num_of_joints_; i++)
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
    as.setPreempted();
    ur_instruction_executor_->cancelMotion();
    send_command_mutex_.unlock();
}

// -----------------------------------------
//                  Main
// -----------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rpwc_ur_bridge_native");
    nh_ = new ros::NodeHandle();
    ros::AsyncSpinner spinner(2);
    spinner.start();

    // DEBUG: Enable debug logs
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

    if (!nh_->getParam("root_name", root_name_))
    {
        ROS_ERROR_STREAM("Param '" << name_space_ << "/root_name' not found on param server");
        set_init_end_status(false, "Param root_name missing");
        return 1;
    }

    if (!nh_->getParam("tip_name", tip_name_))
    {
        ROS_ERROR_STREAM("Param '" << name_space_ << "/tip_name' not found on param server");
        set_init_end_status(false, "Param tip_name missing");
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

        motor_state_on_start_ = true;
        if (check_robot_mode(urcl::RobotMode::POWER_OFF))
        {
            motor_state_on_start_ = false;
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

    std::thread rtde_thread{&thread_handle_rtde};

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

    std::thread joint_states_pub(&thread_pub_joint_states);
    ROS_INFO_STREAM("Started joint_states publisher (ID: " << joint_states_pub.get_id() << ")");

    ur_instruction_executor_.reset(new urcl::InstructionExecutor(ur_driver_));
    ur_primary_ = ur_driver_->getPrimaryClient();

    // Load URDF & init fk chains
    {
        ROS_INFO("Load and parse URDF");
        std::string xml_string;
        if (nh_->hasParam("robot_description"))
            nh_->getParam("robot_description", xml_string);
        else
        {
            ROS_ERROR("Parameter robot_description not set, shutting down node...");
            set_init_end_status(false, "Param robot_description missing");
            return 1;
        }

        if (xml_string.size() == 0)
        {
            ROS_ERROR("Unable to load robot model from parameter robot_description");
            set_init_end_status(false, "Param robot_description invalid");
            return 1;
        }

        // Get urdf model out of robot_description
        urdf::Model model;
        if (!model.initString(xml_string))
        {
            ROS_ERROR("Failed to parse urdf file");
            set_init_end_status(false, "Param  missing");
            return 1;
        }
        ROS_INFO("Successfully parsed urdf file");

        if (!kdl_parser::treeFromUrdfModel(model, kdl_tree_))
        {
            ROS_ERROR("Failed to construct kdl tree");
            set_init_end_status(false, "Param  missing");
            return 1;
        }

        // Populate the KDL chain to EE
        if (!kdl_tree_.getChain(root_name_, tip_name_, kdl_chain_ee_))
        {
            ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
            ROS_ERROR_STREAM("  " << root_name_ << " --> " << tip_name_);
            ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfJoints() << " joints");
            ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfSegments() << " segments");
            ROS_ERROR_STREAM("  The segments are:");

            KDL::SegmentMap segment_map = kdl_tree_.getSegments();
            KDL::SegmentMap::iterator it;

            for (it = segment_map.begin(); it != segment_map.end(); it++)
                ROS_ERROR_STREAM("    " << (*it).first);

            set_init_end_status(false, "Error building kdl_chain_ee_");
            return 1;
        }

        // Populate the KDL chain to LastLink
        std::string ll_name = name_space_ + "/rpwc_last_robot_link";
        ll_name.erase(ll_name.begin());
        if (!kdl_tree_.getChain(root_name_, ll_name, kdl_chain_ll_))
        {
            ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
            ROS_ERROR_STREAM("  " << root_name_ << " --> " << ll_name);
            ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfJoints() << " joints");
            ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfSegments() << " segments");
            ROS_ERROR_STREAM("  The segments are:");

            KDL::SegmentMap segment_map = kdl_tree_.getSegments();
            KDL::SegmentMap::iterator it;

            for (it = segment_map.begin(); it != segment_map.end(); it++)
                ROS_ERROR_STREAM("    " << (*it).first);

            set_init_end_status(false, "Error building kdl_chain_ll_");
            return 1;
        }
        ROS_INFO("KDL Chains ready");

        num_of_joints_ = kdl_chain_ee_.getNrOfJoints();
        q_msr_.resize(num_of_joints_);
        fk_pos_solver_ee_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_ee_));
        fk_pos_solver_ll_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_ll_));
    }

    // Wait for program start
    {
        ros::Time start = ros::Time::now();
        while (rtde_runtime_state_.load(std::memory_order_relaxed) != 2)
        {
            if ((ros::Time::now() - start).toSec() >= 3.0)
            {
                ROS_FATAL("Robot failed to start program within the given timeout (3s)");
                set_init_end_status(false, "Program start failed");
                return 1;
            }
            ros::Duration(0.05).sleep();
        }
    }

    // Load Tool
    {
        double x_pos_ee, y_pos_ee, z_pos_ee, roll_ee, pitch_ee, yaw_ee, roll_last_link, pitch_last_link, yaw_last_link;
        if (!nh_->getParam("x_pos_EE", x_pos_ee))
        {
            ROS_FATAL_STREAM("Param '" << name_space_ << "/x_pos_EE' not found on param server");
            set_init_end_status(false, "Param x_pos_EE missing");
            return 1;
        }

        if (!nh_->getParam("y_pos_EE", y_pos_ee))
        {
            ROS_FATAL_STREAM("Param '" << name_space_ << "/y_pos_EE' not found on param server");
            set_init_end_status(false, "Param y_pos_EE missing");
            return 1;
        }

        if (!nh_->getParam("z_pos_EE", z_pos_ee))
        {
            ROS_FATAL_STREAM("Param '" << name_space_ << "/z_pos_EE' not found on param server");
            set_init_end_status(false, "Param z_pos_EE missing");
            return 1;
        }

        if (!nh_->getParam("roll_EE", roll_ee))
        {
            ROS_FATAL_STREAM("Param '" << name_space_ << "/roll_EE' not found on param server");
            set_init_end_status(false, "Param roll_EE missing");
            return 1;
        }

        if (!nh_->getParam("pitch_EE", pitch_ee))
        {
            ROS_FATAL_STREAM("Param '" << name_space_ << "/pitch_EE' not found on param server");
            set_init_end_status(false, "Param pitch_EE missing");
            return 1;
        }

        if (!nh_->getParam("yaw_EE", yaw_ee))
        {
            ROS_FATAL_STREAM("Param '" << name_space_ << "/yaw_EE' not found on param server");
            set_init_end_status(false, "Param yaw_EE missing");
            return 1;
        }

        if (!nh_->getParam("roll_Last_Link", roll_last_link))
        {
            ROS_FATAL_STREAM("Param '" << name_space_ << "/roll_Last_Link' not found on param server");
            set_init_end_status(false, "Param roll_Last_Link missing");
            return 1;
        }

        if (!nh_->getParam("pitch_Last_Link", pitch_last_link))
        {
            ROS_FATAL_STREAM("Param '" << name_space_ << "/pitch_Last_Link' not found on param server");
            set_init_end_status(false, "Param pitch_Last_Link missing");
            return 1;
        }

        if (!nh_->getParam("yaw_Last_Link", yaw_last_link))
        {
            ROS_FATAL_STREAM("Param '" << name_space_ << "/yaw_Last_Link' not found on param server");
            set_init_end_status(false, "Param yaw_Last_Link missing");
            return 1;
        }

        t_tool02LastLink = KDL::Frame::Identity();
        t_tool02LastLink.M = KDL::Rotation::RPY(roll_last_link, pitch_last_link, yaw_last_link);
        KDL::Frame t_LastLink2EE = KDL::Frame::Identity();
        t_LastLink2EE.p.data[0] = x_pos_ee;
        t_LastLink2EE.p.data[1] = y_pos_ee;
        t_LastLink2EE.p.data[2] = z_pos_ee;
        t_LastLink2EE.M = KDL::Rotation::RPY(roll_ee, pitch_ee, yaw_ee);
        KDL::Frame t_tool02EE = t_tool02LastLink * t_LastLink2EE;
        urcl::vector6d_t tcp_offs;
        tcp_offs[0] = t_tool02EE.p.x();
        tcp_offs[1] = t_tool02EE.p.y();
        tcp_offs[2] = t_tool02EE.p.z();
        tcp_offs[3] = t_tool02EE.M.GetRot().x();
        tcp_offs[4] = t_tool02EE.M.GetRot().y();
        tcp_offs[5] = t_tool02EE.M.GetRot().z();

        if (!ur_driver_->setTcpOffset(tcp_offs))
        {
            ROS_FATAL_STREAM("Failed to set tcp offset");
            set_init_end_status(false, "TCP set failed");
            return 2;
        }
    }

    // Load Payload
    {
        double payload;
        if (!nh_->getParam("mass", payload))
        {
            ROS_FATAL_STREAM("Param '" << name_space_ << "/mass' not found on param server");
            set_init_end_status(false, "Param mass missing");
            return 1;
        }

        std::vector<double> cog;
        if (!nh_->getParam("cog", cog))
        {
            ROS_FATAL_STREAM("Param '" << name_space_ << "/cog' not found on param server");
            set_init_end_status(false, "Param cog missing");
            return 1;
        }
        if (cog.size() != 3)
        {
            ROS_FATAL_STREAM("Param '" << name_space_ << "/cog' must contain exactly 3 elements");
            set_init_end_status(false, "Param cog invalid");
            return 1;
        }

        urcl::vector3d_t cog_ur;
        KDL::Vector p_last(cog[0], cog[1], cog[2]);
        KDL::Vector p_tool0 = t_tool02LastLink * p_last;
        cog_ur[0] = p_tool0.x();
        cog_ur[1] = p_tool0.y();
        cog_ur[2] = p_tool0.z();
        if (!ur_driver_->setPayload(payload, cog_ur))
        {
            ROS_FATAL_STREAM("Failed to set payload");
            set_init_end_status(false, "Payload set failed");
            return 2;
        }
    }

    if (!ur_driver_->getRTDEWriter().sendSpeedSlider(speed_override_))
    {
        ROS_FATAL("Failed to set speed slider");
        set_init_end_status(false, "speed slider set failed");
        return 2;
    }

    // Advertise publishers and services
    std::thread robot_curr_pose_pub(&thread_pub_rob_curr_pose);
    ROS_INFO_STREAM("Started rpwc_robot_curr_pose publisher (ID: " << robot_curr_pose_pub.get_id() << ")");

    std::thread io_signals_state_pub(&thread_pub_io_signals_state);
    ROS_INFO_STREAM("Started io_signals_state_pub publisher (ID: " << io_signals_state_pub.get_id() << ")");

    ros::ServiceServer set_controller_srv = nh_->advertiseService<rpwc_msgs::setController::RequestType, rpwc_msgs::setController::ResponseType>("rpwc_controller", &callback_set_controller);
    ros::ServiceServer srv_get_controller = nh_->advertiseService<rpwc_msgs::getController::RequestType, rpwc_msgs::getController::ResponseType>("get_rpwc_controller", &callback_get_controller);
    ros::ServiceServer set_free_jog_params_srv = nh_->advertiseService<rpwc_msgs::setFreeJogParams::RequestType, rpwc_msgs::setFreeJogParams::ResponseType>("set_free_jog_params", &callback_set_free_jog_params);
    ros::ServiceServer get_free_jog_params_srv = nh_->advertiseService<rpwc_msgs::getFreeJogParams::RequestType, rpwc_msgs::getFreeJogParams::ResponseType>("get_free_jog_params", &callback_get_free_jog_params);
    ros::ServiceServer set_digital_io_signal_srv = nh_->advertiseService<rpwc_msgs::setDigitalIOSignal::RequestType, rpwc_msgs::setDigitalIOSignal::ResponseType>("set_digital_io_signal", &callback_set_digital_io_signal);
    ros::ServiceServer get_digital_io_signal_srv = nh_->advertiseService<rpwc_msgs::getDigitalIOSignal::RequestType, rpwc_msgs::getDigitalIOSignal::ResponseType>("get_digital_io_signal", &callback_get_digital_io_signal);
    ros::ServiceServer set_speed_override_srv = nh_->advertiseService<rpwc_msgs::setSpeedOverride::RequestType, rpwc_msgs::setSpeedOverride::ResponseType>("set_speed_override", &callback_set_speed_override);
    ros::ServiceServer get_speed_override_srv = nh_->advertiseService<rpwc_msgs::getSpeedOverride::RequestType, rpwc_msgs::getSpeedOverride::ResponseType>("get_speed_override", &callback_get_speed_override);
    ros::ServiceServer set_payload_srv = nh_->advertiseService<rpwc_msgs::setPayload::RequestType, rpwc_msgs::setPayload::ResponseType>("rpwc_set_payload", &callback_set_payload);

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
