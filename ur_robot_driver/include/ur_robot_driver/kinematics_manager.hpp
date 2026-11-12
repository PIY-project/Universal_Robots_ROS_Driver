#ifndef UR_KINEMATICS_MANAGER_HPP
#define UR_KINEMATICS_MANAGER_HPP

// -----------------------------------------
//                Includes
// -----------------------------------------
// System includes
#include <atomic>
#include <array>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <eigen3/Eigen/Dense>

// Ros includes
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

// Packages includes
#include <rpwc_msgs/robotArmState.h>
#include <rpwc_msgs/setPayload.h>

// UR Client Library includes
#include <ur_client_library/types.h>
#include <ur_client_library/ur/ur_driver.h>

// -----------------------------------------
//                  Class
// -----------------------------------------

class KinematicsManager
{
public:
    struct RobotConfig
    {
        urcl::vector6d_t tcp_offset{};
        double mass = 0.0;
        urcl::vector3d_t cog{};
        KDL::Frame last_link_to_ee;
    };

    struct PreparedPose
    {
        KDL::Frame base_to_last_link;
        KDL::Frame base_to_ee;
        KDL::JntArray joints;
    };

    /**
     * @brief Parses the URDF, builds KDL chains, creates FK solvers, and loads and
     *        precomputes all tool and payload configuration from ROS parameters.
     *
     * No hardware calls are made. Call applyRobotConfig() once the robot program
     * is running to send the precomputed configuration to the driver.
     *
     * @param nh     ROS node handle used for parameter reads and service advertisement.
     * @param driver Shared UR driver instance; must outlive this object.
     *
     * @throws std::runtime_error if any required ROS parameter is missing or invalid,
     *         or if URDF parsing or KDL chain construction fails.
     */
    KinematicsManager(ros::NodeHandle &nh, std::shared_ptr<urcl::UrDriver> driver);

    /** @brief Calls stop(). */
    ~KinematicsManager();

    /** Sends the last verified configuration when the URScript program starts. */
    void applyRobotConfig();

    /** Uses the same tool0-to-last-link transform as the startup parameters. */
    RobotConfig makeRobotConfig(const std::array<double, 3>& translation,
                                const std::array<double, 3>& rpy, double mass,
                                const std::array<double, 3>& cog) const;
    /** Computes a candidate pose from verified RTDE joints without changing active state. */
    PreparedPose prepareRobotPose(const RobotConfig& config,
                                  const urcl::vector6d_t& joints) const;
    /** Hold this lock through a runtime write and RTDE verification. */
    std::unique_lock<std::mutex> lockRobotConfig() const;
    /** The supplied lock also serializes writes by the existing payload service. */
    void sendRobotConfig(const RobotConfig& config, const std::unique_lock<std::mutex>& lock);
    /** Makes the verified configuration and its first pose available together. */
    void commitVerifiedRobotConfig(const RobotConfig& config, const PreparedPose& pose,
                                   const std::unique_lock<std::mutex>& lock,
                                   const std::function<bool()>& still_ready);

    /** Marks the sent TCP configuration applied only after controller readback. */
    void markRobotConfigVerified() { config_applied_.store(true); }
    void invalidateRobotConfig();
    bool isRobotConfigVerified() const { return config_applied_.load(); }

    /** The flange-to-TCP pose sent to the UR controller, in m and rotation-vector rad. */
    urcl::vector6d_t expectedTcpOffset() const;

    /**
     * @brief Starts the joint-state and pose publisher threads and advertises services.
     *
     * @param rate_hz Publishing rate in Hz (typically the RTDE frequency).
     *
     * @throws std::runtime_error if applyRobotConfig() has not been called first.
     */
    void start(double rate_hz);

    /** @brief Stops both publisher threads and shuts down service servers. */
    void stop();

    /**
     * @brief Stores raw joint data from the RTDE packet. Called from thread_handle_rtde().
     *
     * Thread-safe; protected by joint_data_mutex_.
     *
     * @param joints Measured joint positions (rad).
     * @param vels   Measured joint velocities (rad/s).
     */
    void updateRtdeJointData(const urcl::vector6d_t &joints, const urcl::vector6d_t &vels);

    /** @brief Returns the number of mechanical joints. Used by move_j() and action servers. */
    int getNumJoints() const { return num_joints_; }

private:
    void jointStatePublisherThread();
    void posePublisherThread();
    void frameToPose(const KDL::Frame& frame, bool& first_quat,
                     Eigen::Quaterniond& quat_old, geometry_msgs::Pose& pose);
    bool callbackRobotCurrPose(rpwc_msgs::robotArmState::Request &req, rpwc_msgs::robotArmState::Response &res);
    bool callbackSetPayload(rpwc_msgs::setPayload::Request &req, rpwc_msgs::setPayload::Response &res);
    
    ros::NodeHandle &nh_;
    std::shared_ptr<urcl::UrDriver> driver_;

    KDL::Tree kdl_tree_;
    KDL::Chain kdl_chain_ll_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_ll_;
    KDL::Frame t_tool02LastLink_;
    RobotConfig config_;
    int num_joints_;

    urcl::vector6d_t rob_joints_, rob_joints_vel_;
    std::mutex joint_data_mutex_;
    mutable std::mutex config_command_mutex_;
    mutable std::mutex pose_mutex_;
    KDL::JntArray q_msr_;
    KDL::JntArray pose_joints_;
    KDL::Frame active_last_link_to_ee_;
    bool have_pose_ = false;

    geometry_msgs::PoseStamped curr_pose_ee_, curr_pose_ll_;
    bool first_quat_ee_, first_quat_ll_;
    Eigen::Quaterniond quat_ee_old_, quat_ll_old_;

    double rate_hz_;
    std::thread joint_states_thread_, pose_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> config_applied_{false};

    ros::ServiceServer robot_curr_pose_srv_;
    ros::ServiceServer set_payload_srv_;

    std::string root_name_, ll_name_;
};

#endif // UR_KINEMATICS_MANAGER_HPP
