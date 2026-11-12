#include <ur_robot_driver/kinematics_manager.hpp>

// System includes
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <initializer_list>
#include <limits>
#include <stdexcept>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

// Ros includes
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

// Packages includes
#include <rpwc_msgs/RobotArmStateStamped.h>

// -----------------------------------------
//            KinematicsManager
// -----------------------------------------

namespace
{
void validateSerializedValue(double value)
{
    const double scaled = value * 1000000.0;
    if (!std::isfinite(scaled) ||
        std::round(scaled) < static_cast<double>(std::numeric_limits<std::int32_t>::min()) ||
        std::round(scaled) > static_cast<double>(std::numeric_limits<std::int32_t>::max()))
        throw std::invalid_argument("Robot configuration value cannot be serialized by URCL");
}
}

KinematicsManager::KinematicsManager(ros::NodeHandle &nh, std::shared_ptr<urcl::UrDriver> driver)
    : nh_(nh), driver_(std::move(driver)), num_joints_(0), first_quat_ee_(true), first_quat_ll_(true), rate_hz_(50.0)
{
    if (!nh_.getParam("root_name", root_name_))
        throw std::runtime_error("Param root_name missing");

    if (!nh_.getParam("last_link_name", ll_name_))
        throw std::runtime_error("Param last_link_name missing");

    double x_pos_ee, y_pos_ee, z_pos_ee, roll_ee, pitch_ee, yaw_ee, roll_last_link, pitch_last_link, yaw_last_link;

    if (!nh_.getParam("x_pos_EE", x_pos_ee))
        throw std::runtime_error("Param x_pos_EE missing");
    if (!nh_.getParam("y_pos_EE", y_pos_ee))
        throw std::runtime_error("Param y_pos_EE missing");
    if (!nh_.getParam("z_pos_EE", z_pos_ee))
        throw std::runtime_error("Param z_pos_EE missing");
    if (!nh_.getParam("roll_EE", roll_ee))
        throw std::runtime_error("Param roll_EE missing");
    if (!nh_.getParam("pitch_EE", pitch_ee))
        throw std::runtime_error("Param pitch_EE missing");
    if (!nh_.getParam("yaw_EE", yaw_ee))
        throw std::runtime_error("Param yaw_EE missing");
    if (!nh_.getParam("roll_Last_Link", roll_last_link))
        throw std::runtime_error("Param roll_Last_Link missing");
    if (!nh_.getParam("pitch_Last_Link", pitch_last_link))
        throw std::runtime_error("Param pitch_Last_Link missing");
    if (!nh_.getParam("yaw_Last_Link", yaw_last_link))
        throw std::runtime_error("Param yaw_Last_Link missing");

    ROS_INFO_STREAM("[TCP]: ROS parameters (translation in m, angles in rad): "
        << std::setprecision(17)
        << "x_pos_EE=" << x_pos_ee << " y_pos_EE=" << y_pos_ee << " z_pos_EE=" << z_pos_ee
        << " roll_EE=" << roll_ee << " pitch_EE=" << pitch_ee << " yaw_EE=" << yaw_ee
        << " roll_Last_Link=" << roll_last_link << " pitch_Last_Link=" << pitch_last_link
        << " yaw_Last_Link=" << yaw_last_link);


    ROS_INFO("Load and parse URDF");
    std::string xml_string;
    if (!nh_.hasParam("robot_description"))
        throw std::runtime_error("Param robot_description missing");

    nh_.getParam("robot_description", xml_string);

    if (xml_string.empty())
        throw std::runtime_error("Param robot_description invalid");

    urdf::Model model;
    if (!model.initString(xml_string))
        throw std::runtime_error("Failed to parse urdf file");

    ROS_INFO("Successfully parsed urdf file");

    if (!kdl_parser::treeFromUrdfModel(model, kdl_tree_))
        throw std::runtime_error("Failed to construct kdl tree");

    if (!kdl_tree_.getChain(root_name_, ll_name_, kdl_chain_ll_))
    {
        ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
        ROS_ERROR_STREAM("  " << root_name_ << " --> " << ll_name_);
        ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfJoints() << " joints");
        ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfSegments() << " segments");
        ROS_ERROR_STREAM("  The segments are:");
        KDL::SegmentMap segment_map = kdl_tree_.getSegments();
        for (auto it = segment_map.begin(); it != segment_map.end(); it++)
            ROS_ERROR_STREAM("    " << (*it).first);
        throw std::runtime_error("Error building kdl_chain_ll_");
    }

    ROS_INFO("KDL Chains ready");

    num_joints_ = kdl_chain_ll_.getNrOfJoints();
    if (num_joints_ != 6)
        throw std::runtime_error("UR bridge requires six mechanical joints");
    q_msr_.resize(num_joints_);
    pose_joints_.resize(num_joints_);
    fk_pos_solver_ll_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_ll_));

    curr_pose_ee_.header.frame_id = root_name_;
    curr_pose_ll_.header.frame_id = root_name_;

   
    t_tool02LastLink_ = KDL::Frame::Identity();
    t_tool02LastLink_.M = KDL::Rotation::RPY(roll_last_link, pitch_last_link, yaw_last_link);
    double mass;
    if (!nh_.getParam("mass", mass))
        throw std::runtime_error("Param mass missing");

    std::vector<double> cog;
    if (!nh_.getParam("cog", cog))
        throw std::runtime_error("Param cog missing");
    if (cog.size() != 3)
        throw std::runtime_error("Param cog invalid");

    config_ = makeRobotConfig({{x_pos_ee, y_pos_ee, z_pos_ee}},
        {{roll_ee, pitch_ee, yaw_ee}}, mass, {{cog[0], cog[1], cog[2]}});
    active_last_link_to_ee_ = config_.last_link_to_ee;
}

KinematicsManager::~KinematicsManager()
{
    stop();
}

void KinematicsManager::applyRobotConfig()
{
    auto lock = lockRobotConfig();
    sendRobotConfig(config_, lock);
}

KinematicsManager::RobotConfig KinematicsManager::makeRobotConfig(
    const std::array<double, 3>& translation, const std::array<double, 3>& rpy,
    double mass, const std::array<double, 3>& cog) const
{
    for (double value : translation)
        if (!std::isfinite(value)) throw std::invalid_argument("TCP translation must be finite");
    for (double value : rpy)
        if (!std::isfinite(value)) throw std::invalid_argument("TCP RPY must be finite");
    for (double value : cog)
        if (!std::isfinite(value)) throw std::invalid_argument("Payload centre of mass must be finite");
    if (!std::isfinite(mass) || mass < 0.0)
        throw std::invalid_argument("Payload mass must be finite and nonnegative");

    RobotConfig result;
    result.last_link_to_ee = KDL::Frame(
        KDL::Rotation::RPY(rpy[0], rpy[1], rpy[2]),
        KDL::Vector(translation[0], translation[1], translation[2]));
    const KDL::Frame tool0_to_ee = t_tool02LastLink_ * result.last_link_to_ee;
    const KDL::Vector cog_tool0 = t_tool02LastLink_ * KDL::Vector(cog[0], cog[1], cog[2]);
    result.tcp_offset = {{tool0_to_ee.p.x(), tool0_to_ee.p.y(), tool0_to_ee.p.z(),
        tool0_to_ee.M.GetRot().x(), tool0_to_ee.M.GetRot().y(), tool0_to_ee.M.GetRot().z()}};
    result.mass = mass;
    result.cog = {{cog_tool0.x(), cog_tool0.y(), cog_tool0.z()}};
    for (double value : result.tcp_offset) validateSerializedValue(value);
    validateSerializedValue(result.mass);
    for (double value : result.cog) validateSerializedValue(value);
    return result;
}

KinematicsManager::PreparedPose KinematicsManager::prepareRobotPose(
    const RobotConfig& config, const urcl::vector6d_t& joints) const
{
    PreparedPose pose;
    pose.joints.resize(num_joints_);
    for (int i = 0; i < num_joints_; ++i)
    {
        if (!std::isfinite(joints[i]))
            throw std::invalid_argument("RTDE joint position must be finite");
        pose.joints(i) = joints[i];
    }

    // The publisher owns its FK solver; a local solver avoids sharing mutable
    // KDL solver state with the publishing thread.
    KDL::ChainFkSolverPos_recursive solver(kdl_chain_ll_);
    if (solver.JntToCart(pose.joints, pose.base_to_last_link) < 0)
        throw std::runtime_error("Cannot compute last-link FK from verified RTDE joints");
    pose.base_to_ee = pose.base_to_last_link * config.last_link_to_ee;
    for (const KDL::Frame* frame : {&pose.base_to_last_link, &pose.base_to_ee})
    {
        for (int i = 0; i < 3; ++i)
        {
            if (!std::isfinite(frame->p(i)))
                throw std::runtime_error("Prepared robot pose has non-finite translation");
            for (int j = 0; j < 3; ++j)
                if (!std::isfinite(frame->M(i, j)))
                    throw std::runtime_error("Prepared robot pose has non-finite rotation");
        }
    }
    return pose;
}

std::unique_lock<std::mutex> KinematicsManager::lockRobotConfig() const
{
    return std::unique_lock<std::mutex>(config_command_mutex_);
}

void KinematicsManager::sendRobotConfig(const RobotConfig& config,
                                        const std::unique_lock<std::mutex>& lock)
{
    if (!lock.owns_lock() || lock.mutex() != &config_command_mutex_)
        throw std::logic_error("Robot configuration lock required");
    invalidateRobotConfig();
    if (!driver_->setTcpOffset(config.tcp_offset))
        throw std::runtime_error("TCP set failed");
    if (!driver_->setPayload(config.mass, config.cog))
        throw std::runtime_error("Payload set failed");
}

void KinematicsManager::commitVerifiedRobotConfig(const RobotConfig& config,
                                                   const PreparedPose& pose,
                                                   const std::unique_lock<std::mutex>& lock,
                                                   const std::function<bool()>& still_ready)
{
    if (!lock.owns_lock() || lock.mutex() != &config_command_mutex_)
        throw std::logic_error("Robot configuration lock required");
    std::lock_guard<std::mutex> pose_lock(pose_mutex_);
    if (!still_ready())
        throw std::runtime_error("UR program or RTDE changed before verified pose commit");
    const ros::Time stamp = ros::Time::now();
    curr_pose_ll_.header.stamp = stamp;
    curr_pose_ee_.header.stamp = stamp;
    frameToPose(pose.base_to_last_link, first_quat_ll_, quat_ll_old_, curr_pose_ll_.pose);
    frameToPose(pose.base_to_ee, first_quat_ee_, quat_ee_old_, curr_pose_ee_.pose);
    pose_joints_ = pose.joints;
    config_ = config;
    active_last_link_to_ee_ = config.last_link_to_ee;
    have_pose_ = true;
    config_applied_.store(true);
}

void KinematicsManager::invalidateRobotConfig()
{
    std::lock_guard<std::mutex> pose_lock(pose_mutex_);
    config_applied_.store(false);
    have_pose_ = false;
}

urcl::vector6d_t KinematicsManager::expectedTcpOffset() const
{
    auto lock = lockRobotConfig();
    return config_.tcp_offset;
}

void KinematicsManager::start(double rate_hz)
{
    if (!config_applied_.load())
        throw std::runtime_error("[KinematicsManager]: applyRobotConfig() must be called before start()");

    rate_hz_ = rate_hz;

    robot_curr_pose_srv_ = nh_.advertiseService("rpwc_robot_curr_pose", &KinematicsManager::callbackRobotCurrPose, this);
    set_payload_srv_ = nh_.advertiseService("rpwc_set_payload", &KinematicsManager::callbackSetPayload, this);

    running_ = true;
    joint_states_thread_ = std::thread(&KinematicsManager::jointStatePublisherThread, this);
    pose_thread_ = std::thread(&KinematicsManager::posePublisherThread, this);
}

void KinematicsManager::stop()
{
    running_ = false;

    robot_curr_pose_srv_.shutdown();
    set_payload_srv_.shutdown();

    if (joint_states_thread_.joinable())
        joint_states_thread_.join();
    if (pose_thread_.joinable())
        pose_thread_.join();
}

void KinematicsManager::updateRtdeJointData(const urcl::vector6d_t &joints, const urcl::vector6d_t &vels)
{
    std::lock_guard<std::mutex> lk(joint_data_mutex_);
    rob_joints_ = joints;
    rob_joints_vel_ = vels;
}

void KinematicsManager::frameToPose(const KDL::Frame& frame, bool& first_quat,
                                    Eigen::Quaterniond& quat_old, geometry_msgs::Pose& pose)
{
    Eigen::Matrix3d orient;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
            orient(i, j) = frame.M(i, j);
    }

    Eigen::Quaterniond quat(orient);
    quat.normalize();

    if (first_quat)
    {
        first_quat = false;
        quat_old = quat;
    }

    double sign_check = quat.w() * quat_old.w() + quat.x() * quat_old.x() + quat.y() * quat_old.y() + quat.z() * quat_old.z();
    if (sign_check < 0.0)
    {
        quat.w() = quat.w() * (-1);
        quat.vec() = quat.vec() * (-1);
    }

    quat_old = quat;
    pose.position.x = frame.p.x();
    pose.position.y = frame.p.y();
    pose.position.z = frame.p.z();
    pose.orientation.w = quat.w();
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
}

void KinematicsManager::jointStatePublisherThread()
{
    ROS_INFO("[joint_states]: Init");

    sensor_msgs::JointState msg;
    msg.name.clear();
    if (!nh_.getParam("ur_hardware_interface/joints", msg.name))
    {
        ROS_FATAL_STREAM("[joint_states]: Parameter '" << nh_.getNamespace() << "/ur_hardware_interface/joints' not found on param server");
        ros::requestShutdown();
        return;
    }

    ros::Rate rate(rate_hz_);
    ros::Publisher pub = nh_.advertise<sensor_msgs::JointState>("joint_states", 1, false);

    ROS_INFO("[joint_states]: Start");

    while (running_ && ros::ok())
    {
        msg.position.clear();
        msg.velocity.clear();
        msg.effort.clear();
        msg.header.stamp = ros::Time::now();

        {
            std::lock_guard<std::mutex> lk(joint_data_mutex_);

            for (int i = 0; i < num_joints_; i++)
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

void KinematicsManager::posePublisherThread()
{
    ROS_INFO("[robot_curr_pose]: Init");

    first_quat_ee_ = true;
    first_quat_ll_ = true;

    double dt_pub_pose;
    nh_.param<double>("dt_pub_pose", dt_pub_pose, 0.02);
    ros::Rate r_HZ(1.0 / dt_pub_pose);

    ros::Publisher pub = nh_.advertise<rpwc_msgs::RobotArmStateStamped>("rpwc_robot_curr_pose", 1);
    rpwc_msgs::RobotArmStateStamped msg;
    std_msgs::Float64 tmp;

    ROS_INFO("[robot_curr_pose]: Start");

    while (running_ && ros::ok())
    {
        KDL::JntArray q_local;
        {
            std::lock_guard<std::mutex> lk(joint_data_mutex_);
            q_local = q_msr_;
        }

        KDL::Frame base_to_last_link;
        if (fk_pos_solver_ll_->JntToCart(q_local, base_to_last_link) < 0)
        {
            ROS_ERROR_THROTTLE(1.0, "[robot_curr_pose]: Last-link FK failed");
            r_HZ.sleep();
            continue;
        }

        {
            std::lock_guard<std::mutex> pose_lock(pose_mutex_);
            if (config_applied_.load())
            {
                const KDL::Frame base_to_ee = base_to_last_link * active_last_link_to_ee_;
                const ros::Time stamp = ros::Time::now();
                curr_pose_ll_.header.stamp = stamp;
                curr_pose_ee_.header.stamp = stamp;
                frameToPose(base_to_last_link, first_quat_ll_, quat_ll_old_, curr_pose_ll_.pose);
                frameToPose(base_to_ee, first_quat_ee_, quat_ee_old_, curr_pose_ee_.pose);
                pose_joints_ = q_local;
                have_pose_ = true;

                msg.poseBaseToEe = curr_pose_ee_;
                msg.poseBaseToLastLink = curr_pose_ll_;
                msg.joint_position.clear();
                for (int i = 0; i < num_joints_; i++)
                {
                    tmp.data = q_local(i);
                    msg.joint_position.push_back(tmp);
                }
                pub.publish(msg);
            }
        }

        r_HZ.sleep();
    }

    ROS_INFO("[robot_curr_pose]: Shutting down");
    pub.shutdown();
}

bool KinematicsManager::callbackRobotCurrPose(rpwc_msgs::robotArmState::Request &req, rpwc_msgs::robotArmState::Response &res)
{
    std::lock_guard<std::mutex> pose_lock(pose_mutex_);
    if (!config_applied_.load() || !have_pose_)
        return false;

    res.poseBaseToEe = curr_pose_ee_;
    res.poseBaseToLastLink = curr_pose_ll_;
    for (int i = 0; i < num_joints_; i++)
    {
        std_msgs::Float64 tmp;
        tmp.data = pose_joints_(i);
        res.joint_position.push_back(tmp);
    }

    return true;
}

bool KinematicsManager::callbackSetPayload(rpwc_msgs::setPayload::Request &req, rpwc_msgs::setPayload::Response &res)
{
    if (req.mass.data < 0.0)
    {
        res.result.data = false;
        res.info.data = "Payload mass cannot be negative";
        return true;
    }
    if (!std::isfinite(req.mass.data) ||
        !std::isfinite(req.center_of_mass.x) ||
        !std::isfinite(req.center_of_mass.y) ||
        !std::isfinite(req.center_of_mass.z))
    {
        res.result.data = false;
        res.info.data = "Payload mass or centre of mass is invalid";
        return true;
    }

    double payload = req.mass.data;
    urcl::vector3d_t cog_ur;
    KDL::Vector p_tool0 = t_tool02LastLink_ * KDL::Vector(req.center_of_mass.x, req.center_of_mass.y, req.center_of_mass.z);
    cog_ur[0] = p_tool0.x();
    cog_ur[1] = p_tool0.y();
    cog_ur[2] = p_tool0.z();

    try
    {
        validateSerializedValue(payload);
        for (double value : cog_ur) validateSerializedValue(value);
    }
    catch (const std::invalid_argument& e)
    {
        res.result.data = false;
        res.info.data = e.what();
        return true;
    }

    auto lock = lockRobotConfig();
    if (!config_applied_.load())
    {
        res.result.data = false;
        res.info.data = "Robot configuration is not verified";
        return true;
    }

    bool ok = false;
    try
    {
        ok = driver_->setPayload(payload, cog_ur);
    }
    catch (const std::exception& e)
    {
        invalidateRobotConfig();
        res.result.data = false;
        res.info.data = e.what();
        return true;
    }
    if (ok)
    {
        config_.mass = payload;
        config_.cog = cog_ur;
    }
    else
        invalidateRobotConfig();
    res.result.data = ok;
    if (!ok)
        res.info.data = "Payload set failed";
    return true;
}
