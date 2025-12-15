#include <ur_robot_driver/rpwc_bridge_native.hpp>

// -----------------------------------------
//                 Threads
// -----------------------------------------

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

void thread_read_rtde_data()
{
  ROS_INFO("[RTDE Reader]: Init");
  ros::Rate rate(freq_rtde_hz_);
  std::unique_ptr<urcl::rtde_interface::DataPackage> data_pkg;
  urcl::vector6d_t robData = urcl::vector6d_t();

  ROS_INFO("[RTDE Reader]: Start");
  ROS_WARN_STREAM("q_msr_: " << q_msr_.rows() << " | qd_msr_: " << qd_msr_.rows() << " | num_of_joints_: " << num_of_joints_);

  while (ros::ok())
  {
    data_pkg = ur_driver_->getDataPackage();

    // Joints
    q_mutex_.lock();
    data_pkg->getData<urcl::vector6d_t>("actual_q", robData);
    for (int i = 0; i < num_of_joints_; i++)
      q_msr_(i) = robData[i];
    data_pkg->getData<urcl::vector6d_t>("actual_qd", robData);
    for (int i = 0; i < num_of_joints_; i++)
      qd_msr_(i) = robData[i];
    q_mutex_.unlock();

    // Wrench
    wrench_mutex_.lock();
    data_pkg->getData<urcl::vector6d_t>("actual_TCP_force", wrench_);
    wrench_mutex_.unlock();

    rate.sleep();
  }

  ROS_INFO("[RTDE Reader]: Shutdown");
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
    msg.header.stamp = ros::Time::now();
    msg.position.clear();
    msg.velocity.clear();
    msg.effort.clear();
    q_mutex_.lock();
    for (int i = 0; i < num_of_joints_; i++)
    {
      msg.position.push_back(q_msr_(i));
      msg.velocity.push_back(qd_msr_(i));
      msg.effort.push_back(0.0);
    }
    q_mutex_.unlock();

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

  ros::Rate r_HZ(1.0 / dt_pub_pose_);

  ros::Publisher pub = nh_->advertise<rpwc_msgs::RobotArmStateStamped>("rpwc_robot_curr_pose", 1);
  ros::ServiceServer server_robot_curr_pose = nh_->advertiseService("rpwc_robot_curr_pose", callback_robot_curr_pose);
  curr_pose_ee_.header.frame_id = root_name_;
  curr_pose_ll_.header.frame_id = root_name_;
  rpwc_msgs::RobotArmStateStamped msg;
  std_msgs::Float32 tmp;

  ROS_INFO("[robot_curr_pose]: Start");

  while (ros::ok())
  {
    q_mutex_.lock();
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
    q_mutex_.unlock();

    r_HZ.sleep();
  }

  ROS_INFO("[robot_curr_pose]: Shutting down");
  pub.shutdown();
  server_robot_curr_pose.shutdown();
}

void thread_pub_wrench()
{
  ROS_INFO("[Wrench]: Init");
  ros::Publisher pub = nh_->advertise<geometry_msgs::Wrench>("wrench", 1);
  ros::Rate rate(freq_rtde_hz_);
  geometry_msgs::Wrench msg = geometry_msgs::Wrench();

  ROS_INFO("[Wrench]: Start");

  while (ros::ok())
  {
    wrench_mutex_.lock();
    msg.force.x = wrench_[0];
    msg.force.y = wrench_[1];
    msg.force.z = wrench_[2];
    msg.torque.x = wrench_[3];
    msg.torque.y = wrench_[4];
    msg.torque.z = wrench_[5];
    wrench_mutex_.unlock();

    pub.publish(msg);

    rate.sleep();
  }

  ROS_INFO("[Wrench]: Shutdown");
  pub.shutdown();
}

// -----------------------------------------
//                Functions
// -----------------------------------------

void fwdKin(std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver, KDL::JntArray q, bool& first_quat, Eigen::Vector3d& pos, Eigen::Quaterniond& quat, Eigen::Quaterniond& quat_old)
{
  Eigen::Matrix3d orient;
  KDL::Frame x;  // Tip pose
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
  nh_->shutdown();
  ur_primary_->commandStop();
  ur_dashboard_->commandPowerOff();
  ur_dashboard_->commandClearOperationalMode();
  ur_dashboard_->disconnect();
  ur_driver_->stopControl();
  ur_primary_->stop();
}

void handleRobotProgramState(bool program_running)
{
  ROS_INFO_STREAM("ProgamState changed: " << (program_running ? "RUNNING" : "STOPPED"));
}

bool exec_traj(std::vector<std::shared_ptr<urcl::control::MotionPrimitive>> waypoints)
{
  std::lock_guard<std::mutex> send_command_lock_guard(send_command_mutex_);
  return ur_instruction_executor_->executeMotion(waypoints);
}

bool move_l(std::vector<geometry_msgs::Pose> waypoints, std::vector<float> velocities, std::vector<float> accelerations, std::vector<float> blending_radiuses)
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

bool move_j(std::vector<KDL::JntArray> waypoints, std::vector<float> velocities, std::vector<float> accelerations, std::vector<float> blending_radiuses)
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

bool callback_set_controller(rpwc_msgs::setController::Request& req, rpwc_msgs::setController::Response& res)
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
  res.result.data = ur_driver_->writeFreedriveControlMessage(urcl::control::FreedriveControlMessage::FREEDRIVE_START);
  return true;
}

bool callback_get_controller(rpwc_msgs::getController::Request& req, rpwc_msgs::getController::Response& res)
{
  res.controller = last_controller_started_;
  return true;
}

bool callback_robot_curr_pose(rpwc_msgs::robotArmState::Request& req, rpwc_msgs::robotArmState::Response& res)
{
  res.poseBaseToEe.header.stamp = ros::Time::now();
  res.poseBaseToEe.header.frame_id = curr_pose_ee_.header.frame_id;
  res.poseBaseToEe.pose = curr_pose_ee_.pose;
  res.poseBaseToLastLink.header.stamp = ros::Time::now();
  res.poseBaseToLastLink.header.frame_id = curr_pose_ll_.header.frame_id;
  res.poseBaseToLastLink.pose = curr_pose_ll_.pose;
  for (int i = 0; i < num_of_joints_; i++)
  {
    std_msgs::Float32 tmp;
    tmp.data = q_msr_(i);
    res.joint_position.push_back(tmp);
  }
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

  for (auto& type : rpwc_goal->types)
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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rpwc_ur_bridge_native");
  nh_ = new ros::NodeHandle();
  ros::AsyncSpinner spinner(2);
  urcl::setLogLevel(urcl::LogLevel::INFO);
  urcl::comm::INotifier notifier;
  last_controller_started_ = 0;
  freedrive_ = false;

  name_space_ = nh_->getNamespace();

  if (!nh_->getParam("robot_ip", robot_ip_))
  {
    ROS_FATAL_STREAM("Param '" << name_space_ << "/robot_ip' not found on param server");
    shutdown("Param robot_ip missing");
    return 1;
  }

  if (!nh_->getParam("root_name", root_name_))
  {
    ROS_ERROR_STREAM("Param '" << name_space_ << "/root_name' not found on param server");
    shutdown("Param root_name missing");
    return 1;
  }

  if (!nh_->getParam("tip_name", tip_name_))
  {
    ROS_ERROR_STREAM("Param '" << name_space_ << "/tip_name' not found on param server");
    shutdown("Param tip_name missing");
    return 1;
  }

  if (!nh_->getParam("urscript_file", urscript_file_path_))
  {
    ROS_ERROR_STREAM("Param '" << name_space_ << "/urscript_file' not found on param server");
    shutdown("Param urscript_file missing");
    return 1;
  }

  if (!nh_->getParam("kinematics/hash", calibration_hash_))
  {
    ROS_ERROR_STREAM("Param '" << name_space_ << "/kinematics/hash' not found on param server");
    shutdown("Param kinematics/hash missing");
    return 1;
  }

  if (!nh_->getParam("max_speed_linear", max_speed_linear_))
  {
    ROS_ERROR_STREAM("Param '" << name_space_ << "/max_speed_linear' not found on param server");
    shutdown("Param max_speed_linear missing");
    return 1;
  }

  if (!nh_->getParam("max_acceleration_linear", max_acceleration_linear_))
  {
    ROS_ERROR_STREAM("Param '" << name_space_ << "/max_acceleration_linear' not found on param server");
    shutdown("Param max_acceleration_linear missing");
    return 1;
  }

  if (!nh_->getParam("max_speed_joint", max_speed_joint_))
  {
    ROS_ERROR_STREAM("Param '" << name_space_ << "/max_speed_joint' not found on param server");
    shutdown("Param max_speed_joint missing");
    return 1;
  }

  if (!nh_->getParam("max_acceleration_joint", max_acceleration_joint_))
  {
    ROS_ERROR_STREAM("Param '" << name_space_ << "/max_acceleration_joint' not found on param server");
    shutdown("Param max_acceleration_joint missing");
    return 1;
  }

  nh_->param<float>("rate_rtde_hz", freq_rtde_hz_, 50.0);
  nh_->param<double>("dt_pub_pose", dt_pub_pose_, 0.02);

  ROS_INFO("Load and parse URDF");

  std::string xml_string;
  if (nh_->hasParam("robot_description"))
    nh_->getParam("robot_description", xml_string);
  else
  {
    ROS_ERROR("Parameter robot_description not set, shutting down node...");
    shutdown("Param robot_description missing");
    return 1;
  }

  if (xml_string.size() == 0)
  {
    ROS_ERROR("Unable to load robot model from parameter robot_description");
    shutdown("Param robot_description invalid");
    return 1;
  }

  // Get urdf model out of robot_description
  urdf::Model model;
  if (!model.initString(xml_string))
  {
    ROS_ERROR("Failed to parse urdf file");
    shutdown("Param  missing");
    return 1;
  }
  ROS_INFO("Successfully parsed urdf file");

  if (!kdl_parser::treeFromUrdfModel(model, kdl_tree_))
  {
    ROS_ERROR("Failed to construct kdl tree");
    shutdown("Param  missing");
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

    shutdown("Error building kdl_chain_ee_");
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

    shutdown("Error building kdl_chain_ll_");
    return 1;
  }

  ROS_INFO("KDL Chains ready");

  num_of_joints_ = kdl_chain_ee_.getNrOfJoints();
  fk_pos_solver_ee_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_ee_));
  fk_pos_solver_ll_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_ll_));
  q_msr_ = KDL::JntArray(num_of_joints_);
  qd_msr_ = KDL::JntArray(num_of_joints_);

  ROS_INFO("Starting Primary");
  auto my_primary = std::make_shared<urcl::primary_interface::PrimaryClient>(robot_ip_, notifier);
  my_primary->start();

  ROS_INFO("Starting Dashboard");
  ur_dashboard_.reset(new urcl::DashboardClient(robot_ip_));
  if (!ur_dashboard_->connect(3, std::chrono::seconds(5)))
  {
    URCL_LOG_ERROR("Could not connect to dashboard");
    return 1;
  }

  timeval timeout;
  timeout.tv_sec = 3;
  timeout.tv_usec = 0;
  ur_dashboard_->setReceiveTimeout(timeout);

  // ur_dashboard_->commandPowerOff();
  ur_dashboard_->commandClearOperationalMode();
  // ur_dashboard_->commandPowerOn();
  my_primary->commandBrakeRelease();
  my_primary->stop();

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
  ur_driver_->startRTDECommunication();
  ROS_INFO_STREAM("ControlFrequency: " << ur_driver_->getControlFrequency());

  std::thread read_rtde_data {&thread_read_rtde_data};
  ROS_INFO_STREAM("Started RTDE data reader thread [ID: " << read_rtde_data.get_id() << " ]");

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

  // Load Tool
  double x_pos_ee, y_pos_ee, z_pos_ee, roll_ee, pitch_ee, yaw_ee, roll_last_link, pitch_last_link, yaw_last_link;
  if (!nh_->getParam("x_pos_EE", x_pos_ee))
  {
    ROS_FATAL_STREAM("Param '" << name_space_ << "/x_pos_EE' not found on param server");
    shutdown("Param x_pos_EE missing");
    return 1;
  }

  if (!nh_->getParam("y_pos_EE", y_pos_ee))
  {
    ROS_FATAL_STREAM("Param '" << name_space_ << "/y_pos_EE' not found on param server");
    shutdown("Param y_pos_EE missing");
    return 1;
  }

  if (!nh_->getParam("z_pos_EE", z_pos_ee))
  {
    ROS_FATAL_STREAM("Param '" << name_space_ << "/z_pos_EE' not found on param server");
    shutdown("Param z_pos_EE missing");
    return 1;
  }

  if (!nh_->getParam("roll_EE", roll_ee))
  {
    ROS_FATAL_STREAM("Param '" << name_space_ << "/roll_EE' not found on param server");
    shutdown("Param roll_EE missing");
    return 1;
  }

  if (!nh_->getParam("pitch_EE", pitch_ee))
  {
    ROS_FATAL_STREAM("Param '" << name_space_ << "/pitch_EE' not found on param server");
    shutdown("Param pitch_EE missing");
    return 1;
  }

  if (!nh_->getParam("yaw_EE", yaw_ee))
  {
    ROS_FATAL_STREAM("Param '" << name_space_ << "/yaw_EE' not found on param server");
    shutdown("Param yaw_EE missing");
    return 1;
  }

  if (!nh_->getParam("roll_Last_Link", roll_last_link))
  {
    ROS_FATAL_STREAM("Param '" << name_space_ << "/roll_Last_Link' not found on param server");
    shutdown("Param roll_Last_Link missing");
    return 1;
  }

  if (!nh_->getParam("pitch_Last_Link", pitch_last_link))
  {
    ROS_FATAL_STREAM("Param '" << name_space_ << "/pitch_Last_Link' not found on param server");
    shutdown("Param pitch_Last_Link missing");
    return 1;
  }

  if (!nh_->getParam("yaw_Last_Link", yaw_last_link))
  {
    ROS_FATAL_STREAM("Param '" << name_space_ << "/yaw_Last_Link' not found on param server");
    shutdown("Param yaw_Last_Link missing");
    return 1;
  }

  KDL::Frame t_tool02LastLink = KDL::Frame::Identity();
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

  if (!ur_driver_->setTcp(tcp_offs))
  {
    ROS_FATAL_STREAM("Failed to set tcp offset");
    shutdown("TCP set failed");
    return 2;
  }

  // Load Payload
  double payload;
  if (!nh_->getParam("mass", payload))
  {
    ROS_FATAL_STREAM("Param '" << name_space_ << "/mass' not found on param server");
    shutdown("Param mass missing");
    return 1;
  }

  std::vector<double> cog;
  if (!nh_->getParam("cog", cog))
  {
    ROS_FATAL_STREAM("Param '" << name_space_ << "/cog' not found on param server");
    shutdown("Param cog missing");
    return 1;
  }

  urcl::vector3d_t cog_ur;
  cog_ur[0] = cog[0];
  cog_ur[1] = cog[1];
  cog_ur[2] = cog[2];
  if (!ur_driver_->setPayload(payload, cog_ur))
  {
    ROS_FATAL_STREAM("Failed to set payload");
    shutdown("Payload set failed");
    return 2;
  }

  // Advertise publishers and services
  std::thread joint_states_pub(&thread_pub_joint_states);
  ROS_INFO_STREAM("Started joint_states publisher thread [ID: " << joint_states_pub.get_id() << "]");

  std::thread robot_curr_pose_pub(&thread_pub_rob_curr_pose);
  ROS_INFO_STREAM("Started rpwc_robot_curr_pose publisher thread [ID: " << robot_curr_pose_pub.get_id() << "]");

  std::thread wrench_pub(&thread_pub_wrench);
  ROS_INFO_STREAM("Started wrench publisher thread [ID: " << wrench_pub.get_id() << "]");

  ros::ServiceServer set_controller_srv = nh_->advertiseService<rpwc_msgs::setController::RequestType, rpwc_msgs::setController::ResponseType>("rpwc_controller", &callback_set_controller);
  ros::ServiceServer srv_get_controller = nh_->advertiseService<rpwc_msgs::getController::RequestType, rpwc_msgs::getController::ResponseType>("get_rpwc_controller", &callback_get_controller);

  CartesianMove cart_act_srv("native_cartesian_commands");
  JointsMove joint_act_srv("native_joints_commands");

  spinner.start();
  std::thread keep_alive(&thread_keep_alive);
  ros::waitForShutdown();

  ROS_INFO("Exiting");
  spinner.stop();
  shutdown("Node shutdown");
  return 0;
}
