// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <urdf/model.h>

#include <array>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <eigen3/Eigen/Dense>
// #include <franka/robot_state.h>
// #include <franka_hw/franka_model_interface.h>
// #include <franka_hw/franka_state_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Empty.h>
#include <rpwc_msgs/checkTrajectoryReq.h>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>


#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>

#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>




namespace ur_controllers {

class JointPositionOneTaskInvKin : public controller_interface::MultiInterfaceController<
                                           hardware_interface::PositionJointInterface>  {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  // ros::ServiceClient collBehaviourClient;
  // franka_msgs::SetFullCollisionBehavior collBehaviourSrvMsg;;

 private:
  void fwdKin(KDL::JntArray q, bool &first_quat, Eigen::Vector3d &pos, Eigen::Quaterniond &quat, Eigen::Quaterniond &quat_old);
  bool callback_check_trajectory(rpwc_msgs::checkTrajectoryReq::Request &req, rpwc_msgs::checkTrajectoryReq::Response &res);
  void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  // bool GoHomeCallback(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);
  // double calcPhase(double curr_time, double tau, double alpha);


  hardware_interface::PositionJointInterface* position_joint_interface_;
  hardware_interface::VelocityJointInterface* velocity_joint_interface_;
  // franka_hw::FrankaStateInterface* state_interface_;
  // franka_hw::FrankaModelInterface* model_interface_;

  std::vector<hardware_interface::JointHandle> position_joint_handles_;
  std::vector<hardware_interface::JointHandle> velocity_joint_handles_;
  // std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  // std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;

  std::string name_space_, robot_description_, root_name_, tip_name_;
  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
  boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
  KDL::JntArray  q_;            // Joint computed positions
  KDL::Jacobian  J_;            // Jacobian
  KDL::Frame     x_;            // Tip pose        
  int num_of_joints_;
  bool first_quat_;

  Eigen::VectorXd q_des_;
  Eigen::Vector3d pos_d_;
	Eigen::Quaterniond quat_d_, quat_old_;
  Eigen::MatrixXd k_;
  // ros::Duration elapsed_time_;
  ros::Subscriber sub_pose_des_;
  ros::ServiceServer server_check_trajectory_;

  struct limits_
  {
    KDL::JntArray min;
    KDL::JntArray max;
    KDL::JntArray center;
  } joint_limits_;

  double loop_hz_;



};

}  // namespace ur_controllers