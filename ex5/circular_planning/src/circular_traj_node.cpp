

#include <ros/package.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <angles/angles.h>
#include <moveit_dp_redundancy_resolution/workspace_trajectory.h>
#include <moveit/planning_pipeline/planning_pipeline.h>

void computeVectorsDifference(
    Eigen::VectorXd & diff,
    const Eigen::VectorXd & minuend,
    const Eigen::VectorXd & subtrahend,
    const moveit::core::JointModelGroup * jmg);

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "circular_traj_node");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();


    static const std::string PLANNING_GROUP = "fanuc";
    moveit::planning_interface::MoveGroupInterface move_group("fanuc");

    //const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // Load the robot model
    robot_model_loader::RobotModelLoaderConstPtr robot_model_loader(new robot_model_loader::RobotModelLoader("robot_description"));

    // Get the robot kinematic model
    robot_model::RobotModelConstPtr kinematic_model = robot_model_loader->getModel();

    // Get the planning group
    const robot_state::JointModelGroup * joint_model_group = kinematic_model->getJointModelGroup("fanuc");

    // Compose trajectory path
    std::string package_path = ros::package::getPath("circular_planning");
    std::string filepath = package_path + "/data/fanuc_circular.traj";
    
    moveit_dp_redundancy_resolution::WorkspaceTrajectory work_space_traj("circular_traj",filepath);
    moveit_dp_redundancy_resolution_msgs::WorkspaceTrajectory ws_trajectory_msg;
    work_space_traj.getWorkspaceTrajectoryMsg(ws_trajectory_msg);

    const std::vector<geometry_msgs::Pose>& waypoints=work_space_traj.getWaypoints();

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    
    ROS_INFO_STREAM("Trajectory duration is " << work_space_traj.getDuration() << " s");

    //Create empty joint-space trajectory
    robot_trajectory::RobotTrajectory robot_trajectory(kinematic_model, joint_model_group);

    // Compute IK and verify limits
    double dt = 0;
    Eigen::VectorXd joint_positions_prev = Eigen::VectorXd::Zero(joint_model_group->getVariableCount());
    Eigen::VectorXd joint_positions_curr = Eigen::VectorXd::Zero(joint_model_group->getVariableCount());
    Eigen::VectorXd joint_velocities_prev = Eigen::VectorXd::Zero(joint_model_group->getVariableCount());
    Eigen::VectorXd joint_velocities_curr = Eigen::VectorXd::Zero(joint_model_group->getVariableCount());
    Eigen::VectorXd joint_accelerations = Eigen::VectorXd::Zero(joint_model_group->getVariableCount());

    for(int i=0; i < ws_trajectory_msg.timestamps.size(); i++)
    {
        moveit::core::RobotState robot_state(kinematic_model);

        if(!robot_state.setFromIK(joint_model_group, ws_trajectory_msg.waypoints[i]))
            ROS_WARN_STREAM("Could not compute IK solution for waypoint " << i);

        if(i > 0)
        {
            dt = ws_trajectory_msg.timestamps[i] - ws_trajectory_msg.timestamps[i-1];

            robot_state.copyJointGroupPositions(joint_model_group, joint_positions_curr);

            computeVectorsDifference(joint_velocities_curr, joint_positions_curr, joint_positions_prev, joint_model_group);
            joint_velocities_curr = joint_velocities_curr/dt;

            joint_accelerations = (joint_velocities_curr - joint_velocities_prev)/dt;

            robot_state.setJointGroupVelocities(joint_model_group, joint_velocities_curr);
            robot_state.setJointGroupAccelerations(joint_model_group, joint_accelerations);
        }

        robot_trajectory.addSuffixWayPoint(robot_state, dt);

        joint_positions_prev = joint_positions_curr;
        joint_velocities_prev = joint_velocities_curr;
    }

    
    moveit_msgs::RobotTrajectory trajectory;

    const double jump_threshold = 0.0;
    const double eef_step = 0.01;

    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    ROS_INFO_NAMED("circular_planning", "Visualizing circular plan  (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    visual_tools.deleteAllMarkers();
    visual_tools.publishPath(waypoints, rvt::GREEN, rvt::SMALL);
    visual_tools.trigger();


    // Publish joint space solution to plot in rqt_multiplot
    ros::Publisher plot_trajectory_publisher = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("plot_planned_trajectory", 10000, true);

    ros::Duration sleep_time(0.05);

    for(int i=0; i < trajectory.joint_trajectory.points.size(); i++)
    {
        trajectory_msgs::JointTrajectoryPoint jtp = trajectory.joint_trajectory.points[i];

        plot_trajectory_publisher.publish(jtp);

        sleep_time.sleep();
    }
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");


    spinner.stop();
    ros::shutdown();
    exit(0);
    


}

void computeVectorsDifference(
    Eigen::VectorXd & diff,
    const Eigen::VectorXd & minuend,
    const Eigen::VectorXd & subtrahend,
    const moveit::core::JointModelGroup * jmg)
{
    for (int i = 0; i < minuend.size(); i++)
    {
        if (jmg->getActiveJointModels()[i]->getType() == robot_model::JointModel::REVOLUTE)
        {
            // Compute difference between revolute joints
            robot_model::VariableBounds bounds = jmg->getParentModel().getVariableBounds(jmg->getVariableNames()[i]);

            if(bounds.position_bounded_)
            {
                angles::shortest_angular_distance_with_limits(
                    angles::normalize_angle(subtrahend[i]), 
                    angles::normalize_angle(minuend[i]), 
                    angles::normalize_angle(bounds.min_position_), 
                    angles::normalize_angle(bounds.max_position_), 
                    diff[i]);
            }
            else
                diff[i] = angles::shortest_angular_distance(subtrahend[i], minuend[i]);
        }
        else if (jmg->getActiveJointModels()[i]->getType() == robot_model::JointModel::PLANAR)
        {
            ROS_ERROR("Planar joints are not currently supported.");
        }
        else
        {
            // ! Other joint model types are included here (PRISMATIC, FIXED, UNKNOWN, etc.)
            diff[i] = minuend[i] - subtrahend[i];
        }
    }
}

