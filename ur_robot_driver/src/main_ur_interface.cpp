/*
***********************************************************************************************************************
* 
* Authors: Gianluca Lentini
* Date:13/02/2023
* Version 1.0
***********************************************************************************************************************
*/
#include <ros/ros.h>

#include <stdio.h>
#include <sys/select.h>



#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ListControllers.h>

#include <std_msgs/Float64MultiArray.h>
#include <ur_robot_driver/srv_ur_controller.h>
#include <rpwc_msgs/offset_ee.h>

int state_ = 0;
int previous_state_ = 0;
ros::ServiceClient client_inv_kin_, client_one_task_inv_kin_set_ee_offset_, client_get_last_started_ctrl_, client_set_freedrive_, client_switch_controller_;

bool callback_controller(ur_robot_driver::srv_ur_controller::Request  &req, ur_robot_driver::srv_ur_controller::Response &res)
{
	//STOP LAST CONTOLLER STARTED
	std_srvs::Trigger lastStartedController;
	if(!client_get_last_started_ctrl_.call(lastStartedController))
	{
		ROS_ERROR("Failed to call service client_get_last_started_ctrl_ IN MAIN_UR_INTERFACE");
		return false;
	}
	else
	{
		if(!lastStartedController.response.message.empty())
		{
			if(lastStartedController.response.message.compare("freedrive") == 0)
			{
				std_srvs::SetBool tmpSrv;
				tmpSrv.request.data = false;
				if(!client_set_freedrive_.call(tmpSrv)) 
				{
					ROS_ERROR("Failed to call service client_set_freedrive_ IN MAIN_UR_INTERFACE");
					return false;
				}
			}
			else
			{
				controller_manager_msgs::SwitchController switchCtrlSrv;
				switchCtrlSrv.request.stop_controllers.clear();
				switchCtrlSrv.request.start_controllers.clear();
				switchCtrlSrv.request.stop_controllers.push_back(lastStartedController.response.message);
				switchCtrlSrv.request.start_controllers.push_back("");
				switchCtrlSrv.request.strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT;
				if(!client_switch_controller_.call(switchCtrlSrv)) 
				{
					ROS_ERROR("Failed to call service client_switch_controller_ IN MAIN_UR_INTERFACE");
					return false;
				}
			}
		}
	}

	//START NEW CONTOLLER
	switch (req.controller)
	{
		case 0:
		{
			/* joint group position : per evitare lo scattino provocato dal joint_group_position_controller prima passo dal scaled_pos_joint_traj_controller*/
			controller_manager_msgs::SwitchController switchCtrlSrv;
			switchCtrlSrv.request.stop_controllers.clear();
			switchCtrlSrv.request.start_controllers.clear();
			switchCtrlSrv.request.stop_controllers.push_back("");
			switchCtrlSrv.request.start_controllers.push_back("scaled_pos_joint_traj_controller");
			switchCtrlSrv.request.strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT;
			if(!client_switch_controller_.call(switchCtrlSrv)) 
			{
				ROS_ERROR("Failed to call service client_switch_controller_ IN MAIN_UR_INTERFACE");
				return false;
			}

			switchCtrlSrv.request.stop_controllers.clear();
			switchCtrlSrv.request.start_controllers.clear();
			switchCtrlSrv.request.stop_controllers.push_back("scaled_pos_joint_traj_controller");
			switchCtrlSrv.request.start_controllers.push_back("");
			switchCtrlSrv.request.strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT;
			if(!client_switch_controller_.call(switchCtrlSrv)) 
			{
				ROS_ERROR("Failed to call service client_switch_controller_ IN MAIN_UR_INTERFACE");
				return false;
			}

			switchCtrlSrv.request.stop_controllers.clear();
			switchCtrlSrv.request.start_controllers.clear();
			switchCtrlSrv.request.stop_controllers.push_back("");
			switchCtrlSrv.request.start_controllers.push_back("joint_group_position_controller");
			switchCtrlSrv.request.strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT;
			if(!client_switch_controller_.call(switchCtrlSrv)) 
			{
				ROS_ERROR("Failed to call service client_switch_controller_ IN MAIN_UR_INTERFACE");
				return false;
			}

			std_srvs::SetBool setInvkin;
			setInvkin.request.data = true;
			if(!client_inv_kin_.call(setInvkin)) 
			{
				ROS_ERROR("Failed to call service client_inv_kin_ IN MAIN_UR_INTERFACE");
				return false;
			}

			break;
		}
		
		case 2:
		{	
			/* freedrive */
			std_srvs::SetBool tmpSrv;
			tmpSrv.request.data = true;
			if(!client_set_freedrive_.call(tmpSrv)) 
			{
				ROS_ERROR("Failed to call service client_set_freedrive_ IN MAIN_UR_INTERFACE");
				return false;
			}

			std_srvs::SetBool setInvkin;
			setInvkin.request.data = false;
			if(!client_inv_kin_.call(setInvkin)) 
			{
				ROS_ERROR("Failed to call service client_inv_kin_ IN MAIN_UR_INTERFACE");
				return false;
			}
			break;
		}
	
		default:
		{
			ROS_WARN_STREAM("CONTROLLORE NON SPECIFICATO IN callback_controller MAIN_UR_INTERFACE");
			break;
		}
		
	}

	return true;
}

bool callback_set_ee_offset(rpwc_msgs::offset_ee::Request &req, rpwc_msgs::offset_ee::Response &res)
{
	rpwc_msgs::offset_ee tmpSrv;
	std_srvs::SetBool setInvkin;

	// Disattivo il calcolo della cinematica inversa per sicurezza
	setInvkin.request.data = false;
	if(!client_inv_kin_.call(setInvkin)) 
	{
		ROS_ERROR("Failed to call service client_inv_kin_ IN MAIN_UR_INTERFACE");
		return false;
	}
	//set della nuova trasformata tra robot last link e EE
	tmpSrv.request.T_last_robot_link_to_EE = req.T_last_robot_link_to_EE;
	if (!client_one_task_inv_kin_set_ee_offset_.call(tmpSrv))ROS_ERROR("Failed to call service client_one_task_inv_kin_set_ee_offset");
	// Riattivo il calcolo della cinematica inversa per sicurezza
	setInvkin.request.data = true;
	//10 tentativi nel caso in cui non è entrato ancora nell callback relativa alla lettura della posizione ai giunti
	for(int i = 0; i < 10; i++)
	{
		if(!client_inv_kin_.call(setInvkin)) 
		{
			ROS_ERROR("Failed to call service client_inv_kin_ IN MAIN_ABB_INTERFACE");
			return false;
		}
		else
		{
			if(setInvkin.response.success) break;
			else ROS_WARN_STREAM("NEW ATTEMP TO SET INVKIN IN MAIN_ABB_INTERFACE, REMAINING ATTEMPTS: " << i);
		}
		usleep(250000);
	}

	return true;
}


int main(int argc, char** argv)
{
	//----------------------------------------------------------
	// Preparations
	//----------------------------------------------------------

	// Initialize the node.
	ros::init(argc, argv, "ur_interface_node");
	ros::NodeHandle nh;
	ros::Rate rate(10);

	ros::ServiceServer srv_controller = nh.advertiseService("rpwc_controller", callback_controller);
	ros::ServiceServer server_set_ee_offset_ = nh.advertiseService("rpwc_interface_set_ee_offset", callback_set_ee_offset);


	client_inv_kin_ = nh.serviceClient<std_srvs::SetBool>("inv_kin");
	client_one_task_inv_kin_set_ee_offset_ = nh.serviceClient<rpwc_msgs::offset_ee>("one_task_inv_kin_set_ee_offset");
	client_get_last_started_ctrl_ = nh.serviceClient<std_srvs::Trigger>("ur_hardware_interface/get_last_started_ctrl");
	client_set_freedrive_ = nh.serviceClient<std_srvs::SetBool>("ur_hardware_interface/set_freedrive");
	client_switch_controller_ = nh.serviceClient<controller_manager_msgs::SwitchController>("controller_manager/switch_controller");
	ros::ServiceClient client_get_controllers_list = nh.serviceClient<controller_manager_msgs::ListControllers>("controller_manager/list_controllers");
	ros::ServiceClient client_flag_first_controller_started = nh.serviceClient<std_srvs::Trigger>("ur_hardware_interface/flag_first_controller_started");
	
	client_inv_kin_.waitForExistence();
	client_get_last_started_ctrl_.waitForExistence();
	client_set_freedrive_.waitForExistence();
	client_switch_controller_.waitForExistence();
	client_get_controllers_list.waitForExistence();
  	client_flag_first_controller_started.waitForExistence();

	std_srvs::Trigger flagFirstControllerStarted;
	if(!client_flag_first_controller_started.call(flagFirstControllerStarted))ROS_ERROR("Failed to call service client_flag_first_controller_started IN MAIN_UR_INTERFACE");

	while(!flagFirstControllerStarted.response.success)
	{
		usleep(1000000);
		ROS_WARN_STREAM("WAITING THE FIRST CONTROLLER IS STARTED");
		if(!client_flag_first_controller_started.call(flagFirstControllerStarted))ROS_ERROR("Failed to call service client_flag_first_controller_started IN MAIN_UR_INTERFACE");
	}

	//il primo controllore non è joint_group_position_controller perchè se lo inserisco direttamente da launch fa uno scattino brutto, quindi partiamo da joint_traj_controller e switch subito
	std_srvs::Trigger lastStartedController;
	if(!client_get_last_started_ctrl_.call(lastStartedController))
	{
		ROS_ERROR("Failed to call service client_get_last_started_ctrl_ IN MAIN_UR_INTERFACE");
		return false;
	}
	else
	{
		if(!lastStartedController.response.message.empty())
		{
			if(lastStartedController.response.message.compare("freedrive") == 0)
			{
				std_srvs::SetBool tmpSrv;
				tmpSrv.request.data = false;
				if(!client_set_freedrive_.call(tmpSrv)) 
				{
					ROS_ERROR("Failed to call service client_set_freedrive_ IN MAIN_UR_INTERFACE");
					return 0;
				}
			}
			else
			{
				controller_manager_msgs::SwitchController switchCtrlSrv;
				switchCtrlSrv.request.stop_controllers.clear();
				switchCtrlSrv.request.start_controllers.clear();
				switchCtrlSrv.request.stop_controllers.push_back(lastStartedController.response.message);
				switchCtrlSrv.request.start_controllers.push_back("");
				switchCtrlSrv.request.strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT;
				if(!client_switch_controller_.call(switchCtrlSrv)) 
				{
					
					ROS_ERROR("Failed to call service client_switch_controller_ IN MAIN_UR_INTERFACE");
					return 0;
				}
			}
		}
	}

	controller_manager_msgs::SwitchController switchCtrlSrv;
	switchCtrlSrv.request.stop_controllers.clear();
	switchCtrlSrv.request.start_controllers.clear();
	switchCtrlSrv.request.stop_controllers.push_back("");
	switchCtrlSrv.request.start_controllers.push_back("joint_group_position_controller");
	switchCtrlSrv.request.strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT;
	if(!client_switch_controller_.call(switchCtrlSrv)) 
	{
		ROS_ERROR("Failed to call service client_switch_controller_ IN MAIN_UR_INTERFACE");
		return 0;
	}

	std_srvs::SetBool setInvkin;
	setInvkin.request.data = true;
	//10 tentativi nel caso in cui non è entrato ancora nell callback relativa alla lettura della posizione ai giunti
	for(int i = 0; i < 10; i++)
	{
		if(!client_inv_kin_.call(setInvkin)) 
		{
			ROS_ERROR("Failed to call service client_inv_kin_ IN MAIN_ABB_INTERFACE");
			return 0;
		}
		else
		{
			if(setInvkin.response.success) break;
			else ROS_WARN_STREAM("NEW ATTEMP TO SET INVKIN IN MAIN_ABB_INTERFACE, REMAINING ATTEMPTS: " << i);
		}
	}


	while (ros::ok())
	{

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
