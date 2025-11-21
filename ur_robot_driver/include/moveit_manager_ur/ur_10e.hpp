#ifndef MOVEIT_MANAGER_UR_10E_ARM_HPP_INCLUDED
#define MOVEIT_MANAGER_UR_10E_ARM_HPP_INCLUDED

// -----------------------------------------
//                Includes
// ----------------------------------------

// System includes

// ROS includes
#include <ros/ros.h>

// Packages includes
#include <rpwc/moveit_manager.h>

// -----------------------------------------
//                 Classes
// -----------------------------------------

class moveitManagerUR10e : public moveitManager
{
  public:
    moveitManagerUR10e();
    ~moveitManagerUR10e();

  protected:
    bool customSingularityRecoveryPlan(const std::vector<geometry_msgs::Pose> &waypointsPose, trajSettings &traj_settings, errorCode &error, planningResult &planning_result, const std::vector<double>* initialJointConfig) override;
    bool checkCustomSingularity(const KDL::JntArray &q) override;
};

#endif // MOVEIT_MANAGER_UR_10E_ARM_HPP_INCLUDED