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
#include <std_msgs/Float64MultiArray.h>
#include <ur_robot_driver/srv_ur_controller.h>
#include <ur_robot_driver/offset_ee.h>

int state_ = 0;
int previous_state_ = 0;
ros::ServiceClient client_bridge_com_, client_inv_kin_, client_one_task_inv_kin_set_ee_offset, client_controller_bridge_set_ee_offset;

bool callback_controller(ur_robot_driver::srv_ur_controller::Request  &req, ur_robot_driver::srv_ur_controller::Response &res)
{
	switch (req.controller)
	{
		case 0:
		{
			/* joint group position */
			break;
		}
		
		case 2:
		{
			/* freedrive
				client per controllare ultimo controllare started
				client per stoppare ultimo controllore started
				client per azionare free drive
			*/
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

bool callback_set_ee_offset(ur_robot_driver::offset_ee::Request &req, ur_robot_driver::offset_ee::Response &res)
{
	ur_robot_driver::offset_ee tmpSrv;
	tmpSrv.request.T_last_robot_link_to_EE = req.T_last_robot_link_to_EE;
	if (!client_one_task_inv_kin_set_ee_offset.call(tmpSrv))ROS_ERROR("Failed to call service client_one_task_inv_kin_set_ee_offset");

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
	ros::Rate rate(100);

	ros::ServiceServer srv_controller = nh.advertiseService("ur_controller", callback_controller);
	ros::ServiceServer server_set_ee_offset_ = nh.advertiseService("ur_interface_set_ee_offset", callback_set_ee_offset);


	client_inv_kin_ = nh.serviceClient<std_srvs::Empty>("inv_kin");
	client_one_task_inv_kin_set_ee_offset = nh.serviceClient<ur_robot_driver::offset_ee>("one_task_inv_kin_set_ee_offset");


	while (ros::ok())
	{


	}

	return 0;
}
