#include <ros/ros.h>
#include <kinematics_service_msgs/GetFKSolution.h>
#include <cstdlib>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit_msgs/GetPositionFK.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "fk_service_client_node");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<kinematics_service_msgs::GetFKSolution>("compute_fk_custom_service");

    kinematics_service_msgs::GetFKSolution fk_srv;

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

    moveit::core::RobotState robot_state(kinematic_model);

    const std::vector<double> joint_positions = {0, 0.341793, 0.00991429, 0, -1.9225, -3.14159};

    const robot_state::JointModelGroup * joint_model_group = kinematic_model->getJointModelGroup("fanuc");

    robot_state.setJointGroupPositions(joint_model_group, joint_positions);

    moveit::core::robotStateToRobotStateMsg(robot_state, fk_srv.request.robot_state);

    if(!client.call(fk_srv)){
        ROS_ERROR("Could not compute forward kinematics 7");
        }
    else{
        ROS_INFO("Compute forward kinematics");
    }
    
    // Convert orientation to RPY
    tf2::Quaternion quaternion;
    tf2::fromMsg(fk_srv.response.end_effector_pose.orientation, quaternion);
    
    tf2::Matrix3x3 matrix(quaternion);
    tf2Scalar roll, pitch, yaw;
    matrix.getRPY(roll, pitch, yaw);

    // Print solution to stdout
    std::ostringstream output_msg;

    output_msg << "Computed forward kinematic solution with custom solver:"  << std::endl;
    output_msg << "Position (XYZ): ["; 
    output_msg << fk_srv.response.end_effector_pose.position.x << ", ";
    output_msg << fk_srv.response.end_effector_pose.position.y << ", ";
    output_msg << fk_srv.response.end_effector_pose.position.z << "]" << std::endl;
    output_msg << "Orientation (RPY): [" << roll << ", " << pitch << ", " << yaw << "]";

    ROS_INFO_STREAM(output_msg.str());

    // Compare result with that of move_group's /compute_fk

    client = nh.serviceClient<moveit_msgs::GetPositionFK>("compute_fk");

    std::vector<std::string> link_names = joint_model_group->getLinkModelNames();

    moveit_msgs::GetPositionFK move_group_fk_service;
    move_group_fk_service.request.header.frame_id = link_names[0];
    move_group_fk_service.request.fk_link_names.push_back(link_names.back());
    moveit::core::robotStateToRobotStateMsg(robot_state, move_group_fk_service.request.robot_state);

    if(!client.call(move_group_fk_service))
        ROS_ERROR("Could not compute forward kinematics");

    // Convert orientation to RPY
    tf2::fromMsg(move_group_fk_service.response.pose_stamped[0].pose.orientation, quaternion);
    
    tf2::Matrix3x3 rotation_matrix(quaternion);
    rotation_matrix.getRPY(roll, pitch, yaw);

    // Print solution to stdout
    std::ostringstream msg;

    msg << "Computed forward kinematic solution with move_group solver:"  << std::endl;
    msg << "Position (XYZ): ["; 
    msg << move_group_fk_service.response.pose_stamped[0].pose.position.x << ", ";
    msg << move_group_fk_service.response.pose_stamped[0].pose.position.y << ", ";
    msg << move_group_fk_service.response.pose_stamped[0].pose.position.z << "]" << std::endl;
    msg << "Orientation (RPY): [" << roll << ", " << pitch << ", " << yaw << "]";

    ROS_INFO_STREAM(msg.str());
    
    ros::shutdown();
    return 0;

}
