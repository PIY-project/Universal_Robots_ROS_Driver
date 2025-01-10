#include <ros/ros.h>
#include <ros/package.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Trigger.h>

#include <controller_manager_msgs/UnloadController.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ListControllers.h>

#include <rpwc_msgs/robotArmState.h>
#include <rpwc_msgs/setController.h>
#include <rpwc_msgs/getController.h>

#include <ur_msgs/set_control_to_freedrive.h>

#include <eigen3/Eigen/Dense>

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>

#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>


#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>

#include <urdf/model.h>

#include <thread>

ros::NodeHandle* node_handle_;
std::string name_space_, root_name_, tip_name_;
double dt_pub_pose_;
int num_of_joints_;
int last_controller_started_;
bool first_quat_msr_, flag_joint_msr_;
std::vector<int> joints_order_= {2, 1, 0, 3, 4, 5};
Eigen::Quaterniond quat_old_msr_;
geometry_msgs::PoseStamped curr_pose_;
KDL::Tree kdl_tree_;
KDL::Chain kdl_chain_;
boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
KDL::JntArray  q_msr_;          // Joint measured positions
ros::ServiceClient client_set_control_to_free_drive_, client_switch_controllers_, client_list_controllers_;

std::vector<std::string> GetControllersRunning()
{
	std::vector<std::string> controllers_running;

	controller_manager_msgs::ListControllers::Request  req;
	controller_manager_msgs::ListControllers::Response res;

	client_list_controllers_.call(req, res);

	for(auto controller : res.controller)
	{
		if(controller.state == "running")
		{
			ROS_INFO("Controller: %s is in state: %s", controller.name.c_str(), controller.state.c_str());
			controllers_running.push_back(controller.name);
		}
	}

	return controllers_running;
}

void SwitchControllers(std::vector<std::string>& controllers_to_start,
						std::vector<std::string>& controllers_to_stop,
						int strictness,
						bool start_asap,
						int timeout)
{
	controller_manager_msgs::SwitchController::Request  req;
	controller_manager_msgs::SwitchController::Response res;

	req.start_controllers = controllers_to_start;
	req.stop_controllers = controllers_to_stop;
	req.start_asap = start_asap;
	req.strictness = req.STRICT;
	req.timeout = 0.0;

	client_switch_controllers_.call(req, res);

	if(!res.ok)
		ROS_ERROR("Impossible to switch controllers");

}


bool callback_set_controller(rpwc_msgs::setController::Request  &req, rpwc_msgs::setController::Response &res)
{
	if(req.controller == last_controller_started_)
	{
		ROS_INFO("THIS CONTROLLER IS ALREADY RUNNING");
    res.result.data = true;
		return true;
	}

	switch (req.controller)
	{
		case 0:
		{
			ROS_INFO("START joint_position_one_task_inv_kin CONTROLLER");

      if(last_controller_started_ == 2) //freedrive
      {
        //client set to egm con specifico controllore joint_position_one_task_inv_kin
        ur_msgs::set_control_to_freedrive srv_set_control;
        srv_set_control.request.set_freedrive = false;
        srv_set_control.request.controller_name = "joint_position_one_task_inv_kin";
        if(!client_set_control_to_free_drive_.call(srv_set_control))
        {
          ROS_ERROR("Failed to call service client_set_control_to_free_drive_");
          res.result.data = false;
          return true;
        }
      }
      else
      {
        //stoppa controllori tranne joint state e attiva joint_position_one_task_inv_kin
        // Getting all running controllers
        std::vector<std::string> controllers_running = GetControllersRunning();

        // Check whether you are starting/stopping joint_state_cotroller
        std::vector<std::string> controllers_to_stop;
        for(std::string controller : controllers_running)
        {
          if(controller != "joint_state_controller" && controller != "speed_scaling_state_controller" && controller != "force_torque_sensor_controller") controllers_to_stop.push_back(controller);
        }

        std::vector<std::string> controllers_to_start;
        controllers_to_start.push_back("joint_position_one_task_inv_kin");

        SwitchControllers(controllers_to_start, controllers_to_stop, 2, false, 0.0);
      }

			break;
		}

		case 1:
		{
			ROS_INFO("START PositionControllers_JointTrajectoryController CONTROLLER");

      if(last_controller_started_ == 2) //freedrive
      {
        //client set to egm con specifico controllore joint_position_one_task_inv_kin
        ur_msgs::set_control_to_freedrive srv_set_control;
        srv_set_control.request.set_freedrive = false;
        srv_set_control.request.controller_name = "scaled_pos_joint_traj_controller";
        if(!client_set_control_to_free_drive_.call(srv_set_control))
        {
          ROS_ERROR("Failed to call service client_set_control_to_free_drive_");
          res.result.data = false;
          return true;
        }
      }
      else
      {
        //stoppa controllori tranne joint state e attiva PositionControllers_JointTrajectoryController
        // Getting all running controllers
        std::vector<std::string> controllers_running = GetControllersRunning();

        // Check whether you are starting/stopping joint_state_cotroller
        std::vector<std::string> controllers_to_stop;
        for(std::string controller : controllers_running)
        {
          if(controller != "joint_state_controller" && controller != "speed_scaling_state_controller" && controller != "force_torque_sensor_controller") controllers_to_stop.push_back(controller);
        }

        std::vector<std::string> controllers_to_start;
        controllers_to_start.push_back("scaled_pos_joint_traj_controller");

        SwitchControllers(controllers_to_start, controllers_to_stop, 2, false, 0.0);
      }
			
			break;
		}

		case 2:
		{
			ROS_INFO("START LEAD-THROUGH");

      ur_msgs::set_control_to_freedrive srv_set_control;
      srv_set_control.request.set_freedrive = true;
      if(!client_set_control_to_free_drive_.call(srv_set_control))
      {
        ROS_ERROR("Failed to call service client_set_control_to_free_drive_");
        res.result.data = false;
        return true;
      }
			
			break;
		}

		default:
			break;
	}

	last_controller_started_ = req.controller;
  res.result.data = true;
	return true;
}

bool callback_get_controller(rpwc_msgs::getController::Request  &req, rpwc_msgs::getController::Response &res)
{
	res.controller = last_controller_started_;
	return true;
}

void callback_joint_states(const sensor_msgs::JointState::ConstPtr &msg)
{
	for (int i = 0; i < num_of_joints_; i++)
	{
    q_msr_(i) = msg->position[joints_order_[i]];
	}

	if (flag_joint_msr_ == false) flag_joint_msr_ = true;
}

bool callback_robot_curr_pose(rpwc_msgs::robotArmState::Request  &req, rpwc_msgs::robotArmState::Response &res)
{
	res.pose.pose = curr_pose_.pose;
	res.pose.header.frame_id = curr_pose_.header.frame_id;
	res.pose.header.stamp = ros::Time::now();
	for(int i = 0; i < q_msr_.rows(); i++)
	{
		std_msgs::Float32 tmp;
		tmp.data =  q_msr_(i);
		res.joint_position.push_back(tmp);
	}
	return true;
}

void fwdKin(KDL::JntArray q, bool &first_quat, Eigen::Vector3d &pos, Eigen::Quaterniond &quat, Eigen::Quaterniond &quat_old)
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


void threadPubRobPose()
{
	Eigen::Vector3d pos_msr;
	Eigen::Quaterniond quat_msr;
  ros::NodeHandle nh;
	ros::Rate r_HZ(1.0/dt_pub_pose_);
  
	ros::Publisher pub_curr_pose = nh.advertise<geometry_msgs::PoseStamped>("rpwc_robot_curr_pose", 1);
	ros::ServiceServer server_robot_curr_pose = nh.advertiseService("rpwc_robot_curr_pose", callback_robot_curr_pose);
	curr_pose_.header.frame_id = root_name_;

	while(ros::ok())
	{
		if(flag_joint_msr_)
		{
			fwdKin(q_msr_, first_quat_msr_, pos_msr, quat_msr, quat_old_msr_);
			curr_pose_.header.stamp = ros::Time::now();
			curr_pose_.pose.position.x = pos_msr.x();
			curr_pose_.pose.position.y = pos_msr.y();
			curr_pose_.pose.position.z = pos_msr.z();
			curr_pose_.pose.orientation.w = quat_msr.w();
			curr_pose_.pose.orientation.x = quat_msr.x();
			curr_pose_.pose.orientation.y = quat_msr.y();
			curr_pose_.pose.orientation.z = quat_msr.z();
			pub_curr_pose.publish(curr_pose_);
		}
		r_HZ.sleep();
	}
}

int main(int argc, char** argv){
  ros::init(argc, argv, "rpwc_omnicore_bridge");

  node_handle_ = new(ros::NodeHandle);

	ros::Subscriber sub_joint_states = node_handle_->subscribe("joint_states", 1, &callback_joint_states);

  ros::ServiceServer srv_set_controller = node_handle_->advertiseService("rpwc_controller", callback_set_controller);
  ros::ServiceServer srv_get_controller = node_handle_->advertiseService("get_rpwc_controller", callback_get_controller);

  client_set_control_to_free_drive_     = node_handle_->serviceClient<ur_msgs::set_control_to_freedrive>("ur_hardware_interface/set_freedrive");
  client_switch_controllers_            = node_handle_->serviceClient<controller_manager_msgs::SwitchController>("controller_manager/switch_controller");
  client_list_controllers_              = node_handle_->serviceClient<controller_manager_msgs::ListControllers> ("controller_manager/list_controllers");


  first_quat_msr_ = true;
  flag_joint_msr_ = false;
  last_controller_started_ = 0;
  

  name_space_ = node_handle_->getNamespace();

  if (!node_handle_->getParam("root_name", root_name_))
  {
      ROS_ERROR_STREAM("rpwc_omnicore_bridge: No root name found on parameter server ("<<name_space_<<"/root_name)");
  }

  if (!node_handle_->getParam("tip_name", tip_name_))
  {
      ROS_ERROR_STREAM("rpwc_omnicore_bridge: No tip name found on parameter server ("<<name_space_<<"/tip_name)");
  }

  if (!node_handle_->getParam("dt_pub_pose", dt_pub_pose_))
	{
		ROS_ERROR_STREAM("rpwc_omnicore_bridge: No dt_pub_pose name found on parameter server (" << name_space_ << "/dt_pub_pose)");
	}
  

  // robot_description_ = name_space_ + "/robot_description";
  // Construct an URDF model from the xml string
  std::string xml_string;

  if (node_handle_->hasParam("robot_description"))
      node_handle_->getParam("robot_description", xml_string);
  else
  {
      ROS_ERROR("Parameter %s not set, shutting down node...", "robot_description");
      node_handle_->shutdown();
  }

  if (xml_string.size() == 0)
  {
      ROS_ERROR("Unable to load robot model from parameter %s","robot_description");
      node_handle_->shutdown();
  }

  //ROS_DEBUG("%s content\n%s", robot_description_.c_str(), xml_string.c_str());

  // Get urdf model out of robot_description
  urdf::Model model;
  if (!model.initString(xml_string))
  {
      ROS_ERROR("Failed to parse urdf file");
      node_handle_->shutdown();
  }
  ROS_INFO("Successfully parsed urdf file");
  
  
  if (!kdl_parser::treeFromUrdfModel(model, kdl_tree_))
  {
      ROS_ERROR("Failed to construct kdl tree");
      node_handle_->shutdown();
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

      for( it=segment_map.begin(); it != segment_map.end(); it++ ) ROS_ERROR_STREAM( "    "<<(*it).first);
  }
  num_of_joints_ = kdl_chain_.getNrOfJoints();
  q_msr_.resize(num_of_joints_);
  fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

  std::thread pubRobPoseThread(&threadPubRobPose);
  pubRobPoseThread.detach();

  ros::spin();

  return 0;
};


