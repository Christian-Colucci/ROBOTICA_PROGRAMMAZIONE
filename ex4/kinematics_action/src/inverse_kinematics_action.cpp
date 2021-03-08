/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   inverse_kinematics_action.cpp
 * Author:  Enrico Ferrentino
 * Org.:    UNISA
 * Date:    Oct 9, 2020
 *
 * Refer to the header file for a description of this module.
 * 
 * -------------------------------------------------------------------
 */

#include <kinematics_action/inverse_kinematics_action.h>
#include <moveit/robot_state/conversions.h>
#include <angles/angles.h>

kinematics_action::InverseKinematicsAction::InverseKinematicsAction():
    action_server_(nh_, "ik_solver", boost::bind(&kinematics_action::InverseKinematicsAction::ik_callback_, this, _1), false),
    robot_model_loader_(new robot_model_loader::RobotModelLoader("robot_description")),
    kinematic_model_(robot_model_loader_->getModel()),
    jmg_(nullptr)
{
    // Get the planning group name from the parameter server
    std::string planning_group_name;

    if(!nh_.getParam("planning_group_name", planning_group_name))
    {
        ROS_ERROR("'planning_group_name' is undefined on the parameter server");
        return;
    }

    // Get the planning group
    jmg_ = kinematic_model_->getJointModelGroup(planning_group_name);

    action_server_.start();
}

kinematics_action::InverseKinematicsAction::~InverseKinematicsAction()
{  
}

void kinematics_action::InverseKinematicsAction::ik_callback_(const kinematics_action_msgs::GetIKSolutionGoalConstPtr & goal)
{
    ROS_INFO("IK solver: goal received");

    // Joint model group solver instance
    const kinematics::KinematicsBaseConstPtr solver = jmg_->getSolverInstance();

    kinematics_action_msgs::GetIKSolutionResult result;

    int ik_calls_counter = 0;

    while(ik_calls_counter < 2000 && ros::ok())
    {
        std::vector<double> seed_state = generateSeedState_();
        std::vector<double> solution;
        moveit_msgs::MoveItErrorCodes error_code;

        solver->getPositionIK(goal->end_effector_pose, seed_state, solution, error_code);

        if(error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
            normalizeJointPositions_(solution);

            if(isSolutionNew_(solution))
            {
                ik_solutions_.push_back(solution);

                moveit::core::RobotState robot_state(kinematic_model_);

                robot_state.setVariablePositions(solution);

                kinematics_action_msgs::GetIKSolutionFeedback feedback;

                moveit::core::robotStateToRobotStateMsg(robot_state, feedback.single_robot_state);

                action_server_.publishFeedback(feedback);

                result.robot_state.push_back(feedback.single_robot_state);
            }
        }

        ik_calls_counter++;
    }

    if(ik_solutions_.size() == 0)
        action_server_.setAborted(result, "Could not find any IK solution");
    else
    {
        std::ostringstream ss;
        ss << "Found " << ik_solutions_.size() << " IK solutions";
        
        action_server_.setSucceeded(result, ss.str());
    }

    ik_solutions_.resize(0);
}

bool kinematics_action::InverseKinematicsAction::isSolutionNew_(const std::vector<double> & solution) const
{
    for(int i=0; i < ik_solutions_.size(); i++)
    {
        bool are_solutions_equal = true;

        for(int j=0; j < ik_solutions_[i].size() && are_solutions_equal; j++)
        {
            double diff;

            if(jmg_->getActiveJointModels()[j]->getType() == robot_model::JointModel::REVOLUTE)
            {
                diff = angles::shortest_angular_distance(ik_solutions_[i][j], solution[j]);
            }
            else
            {
                diff = ik_solutions_[i][j] - solution[j];
            }

            if(std::fabs(diff) > 1e-3)
                are_solutions_equal = false;
        }

        if(are_solutions_equal)
            return false;
    }

    return true;
}

std::vector<double> kinematics_action::InverseKinematicsAction::generateSeedState_() const
{
    std::vector<double> seed_state;
    
    std::vector<std::string> joint_names = kinematic_model_->getVariableNames();

    for(int i=0; i < joint_names.size(); i++)
    {
        double lb = kinematic_model_->getURDF()->getJoint(joint_names[i])->limits->lower;
        double ub = kinematic_model_->getURDF()->getJoint(joint_names[i])->limits->upper;
        double span = ub-lb;
        
        seed_state.push_back((double)std::rand()/RAND_MAX * span + lb);
    }

    return seed_state;
}

void kinematics_action::InverseKinematicsAction::normalizeJointPositions_(std::vector<double> & solution) const
{
    for(int i=0; i < solution.size(); i++)
    {
        if (jmg_->getActiveJointModels()[i]->getType() == robot_model::JointModel::REVOLUTE)
        {
            solution[i] = angles::normalize_angle(solution[i]);
        }
    }
}

