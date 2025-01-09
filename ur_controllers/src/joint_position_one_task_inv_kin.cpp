// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <ur_controllers/joint_position_one_task_inv_kin.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "pseudo_inversion.h"
#include "skew_symmetric.h"

namespace ur_controllers {

bool JointPositionOneTaskInvKin::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
    
    name_space_ = node_handle.getNamespace();
    int n = name_space_.find("joint_position_one_task_inv_kin");
    name_space_ = name_space_.substr(0,n);

    if (!node_handle.getParam(name_space_ + "/robot_hw_control_loop/loop_hz", loop_hz_)) {
      ROS_ERROR("JointPositionOneTaskInvKin: Could not parse robot_hw_control_loop/loop_hz");
      return false;
    }

    if (!node_handle.getParam(name_space_ + "/root_name", root_name_))
    {
        ROS_ERROR_STREAM("OneTaskInvKin: No root name found on parameter server ("<<node_handle.getNamespace()<<"/root_name)");
    }

    if (!node_handle.getParam(name_space_ + "/tip_name", tip_name_))
    {
        ROS_ERROR_STREAM("OneTaskInvKin: No tip name found on parameter server ("<<node_handle.getNamespace()<<"/tip_name)");
    }

    

    robot_description_ = name_space_ + "/robot_description";
    // Construct an URDF model from the xml string
    std::string xml_string;

    if (node_handle.hasParam(robot_description_))
        node_handle.getParam(robot_description_.c_str(), xml_string);
    else
    {
        ROS_ERROR("Parameter %s not set, shutting down node...", robot_description_.c_str());
        node_handle.shutdown();
    }

    if (xml_string.size() == 0)
    {
        ROS_ERROR("Unable to load robot model from parameter %s",robot_description_.c_str());
        node_handle.shutdown();
    }

    //ROS_DEBUG("%s content\n%s", robot_description_.c_str(), xml_string.c_str());

    // Get urdf model out of robot_description
    urdf::Model model;
    if (!model.initString(xml_string))
    {
        ROS_ERROR("Failed to parse urdf file");
        node_handle.shutdown();
    }
    ROS_INFO("Successfully parsed urdf file");
    
    
    if (!kdl_parser::treeFromUrdfModel(model, kdl_tree_))
    {
        ROS_ERROR("Failed to construct kdl tree");
        node_handle.shutdown();
    }

    // Populate the KDL chain
    if(!kdl_tree_.getChain(root_name_, tip_name_, kdl_chain_))
    {
        ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
        ROS_ERROR_STREAM("  "<<root_name_<<" --> "<<tip_name_);
        ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfJoints()<<" joints");
        ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfSegments()<<" segments");
        ROS_ERROR_STREAM("  The segments are:");

        KDL::SegmentMap segment_map = kdl_tree_.getSegments();
        KDL::SegmentMap::iterator it;

        for( it=segment_map.begin(); it != segment_map.end(); it++ )
          ROS_ERROR_STREAM( "    "<<(*it).first);

    }

    // Inizializzazione dei limiti per i giunti
    joint_limits_.min.resize(kdl_chain_.getNrOfJoints());
    joint_limits_.max.resize(kdl_chain_.getNrOfJoints());
    joint_limits_.center.resize(kdl_chain_.getNrOfJoints());

    // Otteniamo il link di partenza dal nome del "tip"
    std::shared_ptr<const urdf::Link> link = model.getLink(tip_name_);

    // Ciclo su tutti i giunti della catena KDL
    for (int i = 0; i < kdl_chain_.getNrOfJoints() && link; i++) {
        // Otteniamo il giunto associato al link corrente dal modello URDF
        std::shared_ptr<const urdf::Joint> joint = model.getJoint(link->parent_joint->name);
        
        // Verifica se il giunto è "fixed"
        if (joint->type == urdf::Joint::FIXED) {
            ROS_INFO("Skipping fixed joint: %s", joint->name.c_str());  // Stampa per i giunti fissi
            link = model.getLink(link->getParent()->name);  // Passa al link successivo
            i--;
            continue;  // Salta questa iterazione del ciclo
        }

        // Mostriamo il nome del giunto nei log
        ROS_INFO("Getting limits for joint: %s", joint->name.c_str());

        // Calcoliamo l'indice del giunto nella catena KDL
        int index = kdl_chain_.getNrOfJoints() - i - 1;

        // Memorizziamo i limiti del giunto (min, max, center) negli array di KDL
        joint_limits_.min(index) = joint->limits->lower;
        joint_limits_.max(index) = joint->limits->upper;
        joint_limits_.center(index) = (joint_limits_.min(index) + joint_limits_.max(index)) / 2;

        // Stampa i limiti del giunto
        ROS_INFO("Joint %s limits: min = %f, max = %f, center = %f", 
              joint->name.c_str(), 
              joint_limits_.min(index), 
              joint_limits_.max(index), 
              joint_limits_.center(index));

        // Passiamo al link successivo (genitore del link corrente)
        link = model.getLink(link->getParent()->name);
    }

    ROS_DEBUG("Number of segments: %d", kdl_chain_.getNrOfSegments());
    ROS_DEBUG("Number of joints in chain: %d", kdl_chain_.getNrOfJoints());
    num_of_joints_ = kdl_chain_.getNrOfJoints();


     position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface_ == nullptr) {
    ROS_ERROR(
        "JointPositionOneTaskInvKin: Error getting position joint interface from hardware!");
    return false;
  }

  // velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
  // if (velocity_joint_interface_ == nullptr) {
  //   ROS_ERROR(
  //       "JointPositionOneTaskInvKin: Error getting velocity joint interface from hardware!");
  //   return false;
  // }


  
  double inv_kin_gain_pos;
  double inv_kin_gain_rot;
  if (!node_handle.getParam("inv_kin_gain/pos", inv_kin_gain_pos)) {
    ROS_ERROR("JointPositionOneTaskInvKin: Could not parse inv_kin_gain/pos");
    return false;
  }

  if (!node_handle.getParam("inv_kin_gain/rot", inv_kin_gain_rot)) {
    ROS_ERROR("JointPositionOneTaskInvKin: Could not parse inv_kin_gain/rot");
    return false;
  }


  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joints", joint_names)) {
    ROS_ERROR("JointPositionOneTaskInvKin: Could not parse joint names");
    return false;
  }



  if (joint_names.size() != num_of_joints_) {
    ROS_ERROR_STREAM("JointPositionOneTaskInvKin: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  position_joint_handles_.resize(num_of_joints_);
  // velocity_joint_handles_.resize(num_of_joints_);
  for (size_t i = 0; i < num_of_joints_; ++i) {
    try {
      position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
      // velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "JointPositionOneTaskInvKin: Exception getting joint handles: " << e.what());
      return false;
    }
  }

    jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
    fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

    q_.resize(num_of_joints_);
    J_.resize(num_of_joints_);
    first_quat_ = true;
    q_des_= Eigen::VectorXd::Zero(num_of_joints_);

    k_ = Eigen::MatrixXd::Identity(6, 6);
    k_(0,0) = inv_kin_gain_pos; 
    k_(1,1) = inv_kin_gain_pos; 
    k_(2,2) = inv_kin_gain_pos; 
    k_(3,3) = inv_kin_gain_rot; 
    k_(4,4) = inv_kin_gain_rot; 
    k_(5,5) = inv_kin_gain_rot; 

  //in questo caso utilizziamo name_space_ + "/check_trajectory" perchè vogliamo togliere il nome del controllore dal namespace per essere conpatibiel con il resto del codice di rpwc ma in reatà sarebbe meglio òlasciare anche il namespace del controllore, bisogna prendere una decisione
  sub_pose_des_ = node_handle.subscribe(name_space_ + "/rpwc_pose_des", 1, &JointPositionOneTaskInvKin::PoseCallback, this, ros::TransportHints().reliable().tcpNoDelay());
  //in questo caso utilizziamo name_space_ + "/check_trajectory" perchè vogliamo togliere il nome del controllore dal namespace, cmq questa server andrà tolto perchè si deve utilizzare il joint traj controller
  server_check_trajectory_ = node_handle.advertiseService(name_space_ + "/check_trajectory", &JointPositionOneTaskInvKin::callback_check_trajectory, this);

  return true;
}

void JointPositionOneTaskInvKin::starting(const ros::Time& /* time */) {
  first_quat_ = true;
  for (size_t i = 0; i < num_of_joints_; ++i)
  {
    q_des_(i) = position_joint_handles_[i].getPosition();
    q_(i)=q_des_(i);
  }

  KDL::Frame x; // Tip pose
	// computing forward kinematics

	fk_pos_solver_->JntToCart(q_, x);
	Eigen::Matrix3d orient_d;
	for (int i = 0; i < 3; i++)
	{
		pos_d_(i) = x.p(i);

		for (int j = 0; j < 3; j++)
		{
			orient_d(i, j) = x.M(i, j);
		}
	}
	quat_d_ = orient_d;
	quat_d_.normalize();

}

void JointPositionOneTaskInvKin::fwdKin(KDL::JntArray q, bool &first_quat, Eigen::Vector3d &pos, Eigen::Quaterniond &quat, Eigen::Quaterniond &quat_old)
{
	Eigen::Matrix3d orient;
	KDL::Frame x; // Tip pose
	fk_pos_solver_->JntToCart(q, x);

	for (int i = 0; i < 3; i++)
	{
		pos(i) = x.p(i);

		for (int j = 0; j < 3; j++)
		{
			orient(i, j) = x.M(i, j);
		}
	}

	//from matrix to quat
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

void JointPositionOneTaskInvKin::update(const ros::Time& /*time*/,
                                            const ros::Duration& period) {

  Eigen::Vector3d pos, e_pos, e_quat;
	Eigen::Matrix3d skew;
	Eigen::MatrixXd Jac, Jac_pinv;
	Eigen::Quaterniond quat;
	Eigen::VectorXd e(Eigen::VectorXd::Zero(6));
	Eigen::VectorXd qdot(Eigen::VectorXd::Zero(num_of_joints_));
	KDL::Vector quat_d_vec;
	std_msgs::Float64MultiArray joint_cmd;
	joint_cmd.data.clear();

	// Compute the forward kinematics and Jacobian (at this location).
	fwdKin(q_, first_quat_, pos, quat, quat_old_);
	jnt_to_jac_solver_->JntToJac(q_, J_);

	Jac.resize(6, num_of_joints_);
	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < num_of_joints_; j++)
		{
			Jac(i, j) = J_(i, j);
		}
	}

	// rotation to quaternion issue , "Sign Flip" , check  http://www.dtic.mil/dtic/tr/fulltext/u2/1043624.pdf
  double sign_curr_des;
	double sign_check_curr_des = quat.w() * quat_d_.w() + quat.x() * quat_d_.x() + quat.y() * quat_d_.y() + quat.z() * quat_d_.z();
	if (sign_check_curr_des < 0.0)
		sign_curr_des = (-1.0);
	else
		sign_curr_des = (1.0);

	quat_d_.w() = quat_d_.w() * sign_curr_des;
	quat_d_.vec() = quat_d_.vec() * sign_curr_des;

	quat_d_vec(0) = quat_d_.x();
	quat_d_vec(1) = quat_d_.y();
	quat_d_vec(2) = quat_d_.z();

	skew_symmetric(quat_d_vec, skew);

	e_pos = pos_d_ - pos;
	e_quat = (quat.w() * quat_d_.vec()) - (quat_d_.w() * quat.vec()) - (skew * quat.vec());

	e << e_pos, e_quat;

	pseudoInverse(Jac, Jac_pinv, true);

	qdot = Jac_pinv * k_ *e;

	q_des_ += qdot * period.toSec();

	for (int i = 0; i < num_of_joints_; i++)
	{
    // joint limits saturation
    if (q_des_(i) < joint_limits_.min(i)) q_des_(i) = joint_limits_.min(i);
    if (q_des_(i) > joint_limits_.max(i)) q_des_(i) = joint_limits_.max(i);
		q_(i) = q_des_(i);
    position_joint_handles_[i].setCommand(q_(i));
    // velocity_joint_handles_[i].setCommand(qdot(i));
	}

}

void JointPositionOneTaskInvKin::PoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  pos_d_(0) = msg->pose.position.x;
	pos_d_(1) = msg->pose.position.y;
	pos_d_(2) = msg->pose.position.z;
	quat_d_.w() = msg->pose.orientation.w;
	quat_d_.x() = msg->pose.orientation.x;
	quat_d_.y() = msg->pose.orientation.y;
	quat_d_.z() = msg->pose.orientation.z; 
}

bool JointPositionOneTaskInvKin::callback_check_trajectory(rpwc_msgs::checkTrajectoryReq::Request &req, rpwc_msgs::checkTrajectoryReq::Response &res)
{
	Eigen::Vector3d pos, e_pos, e_quat;
	Eigen::Matrix3d orient, skew;
	Eigen::MatrixXd Jac, Jac_pinv;
	Eigen::Quaterniond quat;
	Eigen::VectorXd e(Eigen::VectorXd::Zero(6));
	Eigen::VectorXd qdot(Eigen::VectorXd::Zero(num_of_joints_));
	KDL::Vector quat_d_Simvec;
	std_msgs::Float64MultiArray joint_cmd;
	
	bool first_quat_sim = true;
	Eigen::Quaterniond quat_old_sim;
	KDL::JntArray q_array;
	KDL::Jacobian J;
	q_array.resize(num_of_joints_);
	Eigen::VectorXd q_eig = Eigen::VectorXd::Zero(num_of_joints_);
	for(int i = 0; i < req.q_start.size(); i++)q_eig(i) = q_array(i) = req.q_start[i].data;
	J.resize(num_of_joints_);
	Jac.resize(6, num_of_joints_);
	int sign_curr_des;
	Eigen::Vector3d pos_d_Sim;
	Eigen::Quaterniond quat_d_Sim;
  double dt = 1.0 / loop_hz_;

	for(int k = 0; k < req.traj.poses.size(); k++)
	{
		pos_d_Sim << req.traj.poses[k].position.x, req.traj.poses[k].position.y, req.traj.poses[k].position.z;
		quat_d_Sim.vec() << req.traj.poses[k].orientation.x, req.traj.poses[k].orientation.y, req.traj.poses[k].orientation.z;
		quat_d_Sim.w() = req.traj.poses[k].orientation.w;
    int num_iteration;
    if(k == (req.traj.poses.size() - 1)) num_iteration = 500;
    else num_iteration = 1;


		for(int times = 0; times < num_iteration; times++)
		{
			
			// Compute the forward kinematics and Jacobian (at this location).
			fwdKin(q_array, first_quat_sim, pos, quat, quat_old_sim);
			jnt_to_jac_solver_->JntToJac(q_array, J);

			for (int i = 0; i < 6; i++)
			{
				for (int j = 0; j < num_of_joints_; j++)
				{
					Jac(i, j) = J(i, j);
				}
			}

			// rotation to quaternion issue , "Sign Flip" , check  http://www.dtic.mil/dtic/tr/fulltext/u2/1043624.pdf
			double sign_check_curr_des = quat.w() * quat_d_Sim.w() + quat.x() * quat_d_Sim.x() + quat.y() * quat_d_Sim.y() + quat.z() * quat_d_Sim.z();
			if (sign_check_curr_des < 0.0)
				sign_curr_des = (-1.0);
			else
				sign_curr_des = (1.0);

			quat_d_Sim.w() = quat_d_Sim.w() * sign_curr_des;
			quat_d_Sim.vec() = quat_d_Sim.vec() * sign_curr_des;

			quat_d_Simvec(0) = quat_d_Sim.x();
			quat_d_Simvec(1) = quat_d_Sim.y();
			quat_d_Simvec(2) = quat_d_Sim.z();

			skew_symmetric(quat_d_Simvec, skew);

			e_pos = pos_d_Sim - pos;
			e_quat = (quat.w() * quat_d_Sim.vec()) - (quat_d_Sim.w() * quat.vec()) - (skew * quat.vec());

			e << e_pos, e_quat;

			pseudoInverse(Jac, Jac_pinv, true);
      

			qdot = Jac_pinv * k_ *e;

			q_eig += qdot * dt;

			joint_cmd.data.clear();
			for (int i = 0; i < num_of_joints_; i++)
			{
				q_array(i) = q_eig(i);
				joint_cmd.data.push_back(q_array(i));

				if((joint_cmd.data[i] < joint_limits_.min(i)) || (joint_cmd.data[i] > joint_limits_.max(i)))
        {
          res.result.data= false;
          return true;
        }
			}
		}
	}
	
	res.q_final.clear();
	for(int i = 0; i < num_of_joints_; i++)
	{
		std_msgs::Float32 tmpFloat;
		tmpFloat.data = joint_cmd.data[i];
		res.q_final.push_back(tmpFloat);
    std::cout << tmpFloat << std::endl;
	}
	res.result.data= true;
	return true;
}

}  // namespace ur_controllers

PLUGINLIB_EXPORT_CLASS(ur_controllers::JointPositionOneTaskInvKin,
                       controller_interface::ControllerBase)