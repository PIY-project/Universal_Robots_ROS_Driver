#ifndef UR_RPWC_BRDIGE_NATIVE_HPP
#define UR_RPWC_BRDIGE_NATIVE_HPP

// -----------------------------------------
//                Includes
// -----------------------------------------
// System includes
#include <iostream>
#include <memory>
#include <sstream>
#include <thread>
#include <mutex>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <eigen3/Eigen/Dense>

// Ros includes
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

// Packages includes
#include <rpwc_msgs/setController.h>
#include <rpwc_msgs/getController.h>
#include <rpwc_msgs/robotArmState.h>
#include <rpwc_msgs/RobotArmStateStamped.h>
#include <rpwc_msgs/nativeCartesianCommandsAction.h>
#include <rpwc_msgs/nativeJointsCommandsAction.h>
#include <rpwc_msgs/setFreeJogParams.h>
#include <rpwc_msgs/getFreeJogParams.h>
#include <rpwc_msgs/setPayload.h>
// UR Client Library includes
#include <ur_client_library/log.h>
#include <ur_client_library/types.h>
#include <ur_client_library/ur/dashboard_client.h>
#include <ur_client_library/ur/datatypes.h>
#include <ur_client_library/ur/ur_driver.h>
#include <ur_client_library/ur/instruction_executor.h>
#include <ur_client_library/ur/robot_receive_timeout.h>
#include <ur_client_library/primary/primary_client.h>
#include <ur_client_library/control/reverse_interface.h>


// -----------------------------------------
//                 Defines
// -----------------------------------------

typedef actionlib::SimpleActionServer<rpwc_msgs::nativeCartesianCommandsAction> CartesianAS;
typedef actionlib::SimpleActionServer<rpwc_msgs::nativeJointsCommandsAction> JointsAS;

// -----------------------------------------
//                Functions
// -----------------------------------------

void thread_keep_alive();
void thread_pub_joint_states();
void thread_pub_rob_curr_pose();
void fwdKin(std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver, KDL::JntArray q, bool& first_quat, Eigen::Vector3d& pos, Eigen::Quaterniond& quat, Eigen::Quaterniond& quat_old);
void shutdown(std::string reason);
void handleRobotProgramState(bool program_running);
bool exec_traj(std::vector<std::shared_ptr<urcl::control::MotionPrimitive>> waypoints);
bool move_l(std::vector<geometry_msgs::Pose> waypoints, std::vector<double> velocities, std::vector<double> accelerations, std::vector<double> blending_radiuses);
bool move_j(std::vector<KDL::JntArray> waypoints, std::vector<double> velocities, std::vector<double> accelerations, std::vector<double> blending_radiuses);

// -----------------------------------------
//           Services Callbacks
// -----------------------------------------

bool callback_set_controller(rpwc_msgs::setController::Request& req, rpwc_msgs::setController::Response& res);
bool callback_get_controller(rpwc_msgs::getController::Request& req, rpwc_msgs::getController::Response& res);
bool callback_robot_curr_pose(rpwc_msgs::robotArmState::Request& req, rpwc_msgs::robotArmState::Response& res);
bool callback_set_free_jog_params(rpwc_msgs::setFreeJogParams::Request& req, rpwc_msgs::setFreeJogParams::Response& res);
bool callback_get_free_jog_params(rpwc_msgs::getFreeJogParams::Request& req, rpwc_msgs::getFreeJogParams::Response& res);

// -----------------------------------------
//             Actions Servers
// -----------------------------------------

class CartesianMove
{
public:
  CartesianMove(std::string name);

  ~CartesianMove(void);

private:
  void goal_callback();
  void preempt_callback();

  CartesianAS as;
  rpwc_msgs::nativeCartesianCommandsGoalConstPtr rpwc_goal;
  rpwc_msgs::nativeCartesianCommandsResult rpwc_result;
  rpwc_msgs::nativeCartesianCommandsFeedback rpwc_feedback;
};

class JointsMove
{
public:
  JointsMove(std::string name);

  ~JointsMove(void);

private:
  void goal_callback();
  void preempt_callback();

  JointsAS as;
  rpwc_msgs::nativeJointsCommandsGoalConstPtr rpwc_goal;
  rpwc_msgs::nativeJointsCommandsResult rpwc_result;
  rpwc_msgs::nativeJointsCommandsFeedback rpwc_feedback;
};

// -----------------------------------------
//               Variables
// -----------------------------------------

ros::NodeHandle* nh_;
std::string name_space_, robot_ip_, root_name_, tip_name_, urscript_file_path_, calibration_hash_;
double freq_rtde_hz_, max_speed_linear_, max_acceleration_linear_, max_speed_joint_, max_acceleration_joint_;
std::shared_ptr<urcl::DashboardClient> ur_dashboard_;
std::shared_ptr<urcl::UrDriver> ur_driver_;
std::shared_ptr<urcl::primary_interface::PrimaryClient> ur_primary_;
std::shared_ptr<urcl::InstructionExecutor> ur_instruction_executor_;
KDL::Tree kdl_tree_;
KDL::Chain kdl_chain_ee_, kdl_chain_ll_;
int num_of_joints_, last_controller_started_;
KDL::JntArray q_msr_;
bool freedrive_, first_quat_ee_msr_, first_quat_ll_msr_;
Eigen::Quaterniond quat_ee_old_msr_, quat_ll_old_msr_;
geometry_msgs::PoseStamped curr_pose_ee_, curr_pose_ll_;
std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_ee_, fk_pos_solver_ll_;
std::mutex send_command_mutex_;
urcl::control::FreedriveParams freedrive_params_;
KDL::Frame t_tool02LastLink = KDL::Frame::Identity();

#endif  // UR_RPWC_BRIDGE_NATIVE_HPP