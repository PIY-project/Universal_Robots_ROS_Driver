#ifndef UR_RPWC_BRDIGE_NATIVE_HPP
#define UR_RPWC_BRDIGE_NATIVE_HPP

// -----------------------------------------
//                Includes
// -----------------------------------------
// System includes
#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

// Ros includes
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>

// Packages includes
#include <rpwc_msgs/checkHardwareStatus.h>
#include <rpwc_msgs/setController.h>
#include <rpwc_msgs/getController.h>
#include <rpwc_msgs/robotArmState.h>
#include <rpwc_msgs/RobotArmStateStamped.h>
#include <rpwc_msgs/nativeCartesianCommandsAction.h>
#include <rpwc_msgs/nativeJointsCommandsAction.h>
#include <rpwc_msgs/setFreeJogParams.h>
#include <rpwc_msgs/getFreeJogParams.h>
#include <rpwc_msgs/setSpeedOverride.h>
#include <rpwc_msgs/getSpeedOverride.h>

// Local includes
#include <ur_robot_driver/urcl_log_handler.h>
#include <ur_robot_driver/io_manager.hpp>
#include <ur_robot_driver/robot_state_manager.hpp>
#include <ur_robot_driver/kinematics_manager.hpp>

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

void set_init_end_status(const bool success, const std::string &msg);
bool check_robot_mode(const urcl::RobotMode robot_mode);
bool check_safety_mode(const urcl::SafetyMode safety_mode);

void thread_handle_rtde();
void thread_keep_alive();
void thread_pub_wrench();

void handleRobotProgramState(bool program_running);
void shutdown(std::string reason);

bool exec_traj(std::vector<std::shared_ptr<urcl::control::MotionPrimitive>> waypoints);
bool move_l(std::vector<geometry_msgs::Pose> waypoints, std::vector<double> velocities, std::vector<double> accelerations, std::vector<double> blending_radiuses);
bool move_j(std::vector<KDL::JntArray> waypoints, std::vector<double> velocities, std::vector<double> accelerations, std::vector<double> blending_radiuses);

// -----------------------------------------
//           Services Callbacks
// -----------------------------------------

bool callback_check_hardware_status(rpwc_msgs::checkHardwareStatus::Request &req, rpwc_msgs::checkHardwareStatus::Response &res);

bool callback_set_controller(rpwc_msgs::setController::Request &req, rpwc_msgs::setController::Response &res);
bool callback_get_controller(rpwc_msgs::getController::Request &req, rpwc_msgs::getController::Response &res);

bool callback_set_free_jog_params(rpwc_msgs::setFreeJogParams::Request &req, rpwc_msgs::setFreeJogParams::Response &res);
bool callback_get_free_jog_params(rpwc_msgs::getFreeJogParams::Request &req, rpwc_msgs::getFreeJogParams::Response &res);

bool callback_set_speed_override(rpwc_msgs::setSpeedOverride::Request &req, rpwc_msgs::setSpeedOverride::Response &res);
bool callback_get_speed_override(rpwc_msgs::getSpeedOverride::Request &req, rpwc_msgs::getSpeedOverride::Response &res);

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

ros::NodeHandle *nh_;
int8_t init_status_;
bool init_success_, enable_wrench_publisher_;
std::string init_msg_, name_space_, robot_ip_, urscript_file_path_, calibration_hash_;
double freq_rtde_hz_, max_speed_linear_, max_acceleration_linear_, max_speed_joint_, max_acceleration_joint_;
std::shared_ptr<urcl::DashboardClient> ur_dashboard_;
std::shared_ptr<urcl::UrDriver> ur_driver_;
std::shared_ptr<urcl::primary_interface::PrimaryClient> ur_primary_;
std::shared_ptr<urcl::InstructionExecutor> ur_instruction_executor_;
std::unique_ptr<IOManager> io_manager_;
std::unique_ptr<RobotStateManager> robot_state_manager_;
std::unique_ptr<KinematicsManager> kinematics_manager_;
int last_controller_started_;
bool motor_off_on_shutdown_, freedrive_;
std::mutex send_command_mutex_, wrench_mutex_;
urcl::control::FreedriveParams freedrive_params_;
double speed_override_;
urcl::vector6d_t ft_raw_wrench_vec_;
std::vector<std::thread> thread_handles_;

#endif // UR_RPWC_BRIDGE_NATIVE_HPP
