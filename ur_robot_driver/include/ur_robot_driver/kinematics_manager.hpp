#ifndef UR_KINEMATICS_MANAGER_HPP
#define UR_KINEMATICS_MANAGER_HPP

// -----------------------------------------
//                Includes
// -----------------------------------------
// System includes
#include <atomic>
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

    /**
     * @brief Sends the precomputed TCP offset and payload to the robot hardware.
     *
     * All values were precomputed in the constructor; this method only performs
     * the driver calls. Requires the URScript program to be running.
     * Must be called before start().
     *
     * @throws std::runtime_error if a driver call fails.
     */
    void applyRobotConfig();

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

    /** @brief Returns the number of joints in the EE kinematic chain. Used by move_j() and action servers. */
    int getNumJoints() const { return num_joints_; }

private:
    void jointStatePublisherThread();
    void posePublisherThread();
    void fwdKin(std::shared_ptr<KDL::ChainFkSolverPos_recursive> solver, KDL::JntArray q, bool &first_quat, Eigen::Vector3d &pos, Eigen::Quaterniond &quat, Eigen::Quaterniond &quat_old);
    bool callbackRobotCurrPose(rpwc_msgs::robotArmState::Request &req, rpwc_msgs::robotArmState::Response &res);
    bool callbackSetPayload(rpwc_msgs::setPayload::Request &req, rpwc_msgs::setPayload::Response &res);

    ros::NodeHandle &nh_;
    std::shared_ptr<urcl::UrDriver> driver_;

    KDL::Tree kdl_tree_;
    KDL::Chain kdl_chain_ee_, kdl_chain_ll_;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_ee_, fk_pos_solver_ll_;
    KDL::Frame t_tool02LastLink_;
    urcl::vector6d_t tcp_offs_;
    double payload_;
    urcl::vector3d_t cog_ur_;
    int num_joints_;

    urcl::vector6d_t rob_joints_, rob_joints_vel_;
    std::mutex joint_data_mutex_;
    KDL::JntArray q_msr_;

    geometry_msgs::PoseStamped curr_pose_ee_, curr_pose_ll_;
    bool first_quat_ee_, first_quat_ll_;
    Eigen::Quaterniond quat_ee_old_, quat_ll_old_;

    double rate_hz_;
    std::thread joint_states_thread_, pose_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> config_applied_{false};

    ros::ServiceServer robot_curr_pose_srv_;
    ros::ServiceServer set_payload_srv_;
};

#endif // UR_KINEMATICS_MANAGER_HPP
