/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   inverse_kinematics_action.h
 * Author:  Enrico Ferrentino
 * Org.:    UNISA
 * Date:    Oct 9, 2020
 *
 * This module implements an action server to compute all the inverse
 * kinematic solutions for a given end-effector pose.
 * 
 * -------------------------------------------------------------------
 */

#ifndef INCLUDE_KINEMATICS_ACTION_INVERSE_KINEMATICS_ACTION_H_
#define INCLUDE_KINEMATICS_ACTION_INVERSE_KINEMATICS_ACTION_H_

#include <ros/ros.h>
#include <kinematics_action_msgs/GetIKSolutionAction.h>
#include <actionlib/server/simple_action_server.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

namespace kinematics_action
{

class InverseKinematicsAction
{

public:
    InverseKinematicsAction();
    ~InverseKinematicsAction();    

private:
    void ik_callback_(const kinematics_action_msgs::GetIKSolutionGoalConstPtr & goal);
    bool isSolutionNew_(const std::vector<double> & solution) const;
    std::vector<double> generateSeedState_() const;
    void normalizeJointPositions_(std::vector<double> & solution) const;

    ros::NodeHandle nh_;
    
    actionlib::SimpleActionServer<kinematics_action_msgs::GetIKSolutionAction> action_server_;

    std::vector<std::vector<double>> ik_solutions_;
    robot_model_loader::RobotModelLoaderConstPtr robot_model_loader_;
    robot_model::RobotModelConstPtr kinematic_model_;
    const robot_state::JointModelGroup * jmg_;
};

} // namespace kinematics_action

#endif // INCLUDE_KINEMATICS_ACTION_INVERSE_KINEMATICS_ACTION_H_