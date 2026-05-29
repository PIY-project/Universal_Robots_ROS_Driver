#include <ur_robot_driver/kinematics_manager.hpp>

// System includes
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

KinematicsManager::KinematicsManager(ros::NodeHandle &nh, std::shared_ptr<urcl::UrDriver> driver)
    : nh_(nh), driver_(std::move(driver)), payload_(0.0), num_joints_(0), first_quat_ee_(true), first_quat_ll_(true), rate_hz_(50.0)
{
    std::string name_space = nh_.getNamespace();
    std::string root_name, tip_name;

    if (!nh_.getParam("root_name", root_name))
        throw std::runtime_error("Param root_name missing");

    if (!nh_.getParam("tip_name", tip_name))
        throw std::runtime_error("Param tip_name missing");

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

    if (!kdl_tree_.getChain(root_name, tip_name, kdl_chain_ee_))
    {
        ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
        ROS_ERROR_STREAM("  " << root_name << " --> " << tip_name);
        ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfJoints() << " joints");
        ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfSegments() << " segments");
        ROS_ERROR_STREAM("  The segments are:");
        KDL::SegmentMap segment_map = kdl_tree_.getSegments();
        for (auto it = segment_map.begin(); it != segment_map.end(); it++)
            ROS_ERROR_STREAM("    " << (*it).first);
        throw std::runtime_error("Error building kdl_chain_ee_");
    }

    std::string ll_name = name_space + "/rpwc_last_robot_link";
    ll_name.erase(ll_name.begin());
    if (!kdl_tree_.getChain(root_name, ll_name, kdl_chain_ll_))
    {
        ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
        ROS_ERROR_STREAM("  " << root_name << " --> " << ll_name);
        ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfJoints() << " joints");
        ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfSegments() << " segments");
        ROS_ERROR_STREAM("  The segments are:");
        KDL::SegmentMap segment_map = kdl_tree_.getSegments();
        for (auto it = segment_map.begin(); it != segment_map.end(); it++)
            ROS_ERROR_STREAM("    " << (*it).first);
        throw std::runtime_error("Error building kdl_chain_ll_");
    }

    ROS_INFO("KDL Chains ready");

    num_joints_ = kdl_chain_ee_.getNrOfJoints();
    q_msr_.resize(num_joints_);
    fk_pos_solver_ee_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_ee_));
    fk_pos_solver_ll_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_ll_));

    curr_pose_ee_.header.frame_id = root_name;
    curr_pose_ll_.header.frame_id = root_name;

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

    t_tool02LastLink_ = KDL::Frame::Identity();
    t_tool02LastLink_.M = KDL::Rotation::RPY(roll_last_link, pitch_last_link, yaw_last_link);
    KDL::Frame t_LastLink2EE = KDL::Frame::Identity();
    t_LastLink2EE.p.data[0] = x_pos_ee;
    t_LastLink2EE.p.data[1] = y_pos_ee;
    t_LastLink2EE.p.data[2] = z_pos_ee;
    t_LastLink2EE.M = KDL::Rotation::RPY(roll_ee, pitch_ee, yaw_ee);
    KDL::Frame t_tool02EE = t_tool02LastLink_ * t_LastLink2EE;
    tcp_offs_[0] = t_tool02EE.p.x();
    tcp_offs_[1] = t_tool02EE.p.y();
    tcp_offs_[2] = t_tool02EE.p.z();
    tcp_offs_[3] = t_tool02EE.M.GetRot().x();
    tcp_offs_[4] = t_tool02EE.M.GetRot().y();
    tcp_offs_[5] = t_tool02EE.M.GetRot().z();

    if (!nh_.getParam("mass", payload_))
        throw std::runtime_error("Param mass missing");

    std::vector<double> cog;
    if (!nh_.getParam("cog", cog))
        throw std::runtime_error("Param cog missing");
    if (cog.size() != 3)
        throw std::runtime_error("Param cog invalid");

    KDL::Vector p_tool0 = t_tool02LastLink_ * KDL::Vector(cog[0], cog[1], cog[2]);
    cog_ur_[0] = p_tool0.x();
    cog_ur_[1] = p_tool0.y();
    cog_ur_[2] = p_tool0.z();
}

KinematicsManager::~KinematicsManager()
{
    stop();
}

void KinematicsManager::applyRobotConfig()
{
    if (!driver_->setTcpOffset(tcp_offs_))
        throw std::runtime_error("TCP set failed");

    if (!driver_->setPayload(payload_, cog_ur_))
        throw std::runtime_error("Payload set failed");

    config_applied_ = true;
}

void KinematicsManager::start(double rate_hz)
{
    if (!config_applied_.load())
        throw std::runtime_error("[KinematicsManager]: applyRobotConfig() must be called before start()");

    rate_hz_ = rate_hz;

    robot_curr_pose_srv_ = nh_.advertiseService("rpwc_robot_curr_pose", &KinematicsManager::callbackRobotCurrPose, this);
    set_payload_srv_ = nh_.advertiseService("rpwc_set_payload", &KinematicsManager::callbackSetPayload, this);

    joint_states_thread_ = std::thread(&KinematicsManager::jointStatePublisherThread, this);
    pose_thread_ = std::thread(&KinematicsManager::posePublisherThread, this);

    running_ = true;
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

void KinematicsManager::fwdKin(std::shared_ptr<KDL::ChainFkSolverPos_recursive> solver, KDL::JntArray q, bool &first_quat, Eigen::Vector3d &pos, Eigen::Quaterniond &quat, Eigen::Quaterniond &quat_old)
{
    Eigen::Matrix3d orient;
    KDL::Frame x;
    solver->JntToCart(q, x);

    for (int i = 0; i < 3; i++)
    {
        pos(i) = x.p(i);

        for (int j = 0; j < 3; j++)
        {
            orient(i, j) = x.M(i, j);
        }
    }

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

    Eigen::Vector3d pos_ee_msr, pos_ll_msr;
    Eigen::Quaterniond quat_ee_msr, quat_ll_msr;
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

        fwdKin(fk_pos_solver_ee_, q_local, first_quat_ee_, pos_ee_msr, quat_ee_msr, quat_ee_old_);
        curr_pose_ee_.header.stamp = ros::Time::now();
        curr_pose_ee_.pose.position.x = pos_ee_msr.x();
        curr_pose_ee_.pose.position.y = pos_ee_msr.y();
        curr_pose_ee_.pose.position.z = pos_ee_msr.z();
        curr_pose_ee_.pose.orientation.w = quat_ee_msr.w();
        curr_pose_ee_.pose.orientation.x = quat_ee_msr.x();
        curr_pose_ee_.pose.orientation.y = quat_ee_msr.y();
        curr_pose_ee_.pose.orientation.z = quat_ee_msr.z();
        fwdKin(fk_pos_solver_ll_, q_local, first_quat_ll_, pos_ll_msr, quat_ll_msr, quat_ll_old_);
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
        for (int i = 0; i < num_joints_; i++)
        {
            tmp.data = q_local(i);
            msg.joint_position.push_back(tmp);
        }
        pub.publish(msg);

        r_HZ.sleep();
    }

    ROS_INFO("[robot_curr_pose]: Shutting down");
    pub.shutdown();
}

bool KinematicsManager::callbackRobotCurrPose(rpwc_msgs::robotArmState::Request &req, rpwc_msgs::robotArmState::Response &res)
{
    res.poseBaseToEe.header.stamp = ros::Time::now();
    res.poseBaseToEe.header.frame_id = curr_pose_ee_.header.frame_id;
    res.poseBaseToEe.pose = curr_pose_ee_.pose;
    res.poseBaseToLastLink.header.stamp = ros::Time::now();
    res.poseBaseToLastLink.header.frame_id = curr_pose_ll_.header.frame_id;
    res.poseBaseToLastLink.pose = curr_pose_ll_.pose;

    {
        std::lock_guard<std::mutex> lk(joint_data_mutex_);
        for (int i = 0; i < num_joints_; i++)
        {
            std_msgs::Float64 tmp;
            tmp.data = q_msr_(i);
            res.joint_position.push_back(tmp);
        }
    }

    return true;
}

bool KinematicsManager::callbackSetPayload(rpwc_msgs::setPayload::Request &req, rpwc_msgs::setPayload::Response &res)
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

    double payload = req.mass.data;
    urcl::vector3d_t cog_ur;
    KDL::Vector p_tool0 = t_tool02LastLink_ * KDL::Vector(req.center_of_mass.x, req.center_of_mass.y, req.center_of_mass.z);
    cog_ur[0] = p_tool0.x();
    cog_ur[1] = p_tool0.y();
    cog_ur[2] = p_tool0.z();

    res.result.data = driver_->setPayload(payload, cog_ur);
    return true;
}
