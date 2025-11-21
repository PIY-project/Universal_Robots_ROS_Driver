#include <moveit_manager_ur/ur_10e.hpp>

// -----------------------------------------
//              Moveit Manager
// -----------------------------------------

moveitManagerUR10e::moveitManagerUR10e() {}

moveitManagerUR10e::~moveitManagerUR10e() {}

bool moveitManagerUR10e::checkCustomSingularity(const KDL::JntArray &q)
{
  return true;
}

bool moveitManagerUR10e::customSingularityRecoveryPlan(const std::vector<geometry_msgs::Pose> &waypointsPose, trajSettings &traj_settings, errorCode &error, planningResult &planning_result, const std::vector<double>* initialJointConfig)
{
  ROS_ERROR("No custom singularity defined");
  return false;
}


// -----------------------------------------
//                  Main
// -----------------------------------------

int main(int argc, char **argv)
{
  ros::init(argc, argv, "moveit_manager_ur_10e");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();

  std::string robot_id{nh.getNamespace()};

  ROS_INFO_STREAM("Start Moveit Manager UR 10e for robot: " << robot_id);
  moveitManagerUR10e mmg5{};

  ros::waitForShutdown();
  return 0;
}