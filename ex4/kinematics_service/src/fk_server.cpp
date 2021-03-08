#include <ros/ros.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <eigen_conversions/eigen_msg.h>
#include <kinematics_service_msgs/GetFKSolution.h>

bool compute_fk(kinematics_service_msgs::GetFKSolutionRequest & request, kinematics_service_msgs::GetFKSolutionResponse & response);

int main(int argc, char**argv){

    ros::init(argc, argv, "fk_service_server_node");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("compute_fk_custom_service", compute_fk);

    ROS_INFO("Started FK service ");
    ros::spin();
    ros::shutdown();
    return 0;

}

bool compute_fk(kinematics_service_msgs::GetFKSolutionRequest & request ,kinematics_service_msgs::GetFKSolutionResponse & response)
{

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    moveit::core::RobotState robot_state(kinematic_model);
    moveit::core::robotStateMsgToRobotState(request.robot_state,robot_state);

    robot_state::RobotStatePtr kinematic_state= std::make_shared<robot_state::RobotState>(robot_state);

    const robot_state::JointModelGroup * joint_model_group = kinematic_model->getJointModelGroup("fanuc");

    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }

    std::vector<std::string> link_names = joint_model_group->getLinkModelNames();

    for (std::size_t i = 0; i < link_names.size(); ++i)
    {
        ROS_INFO("Link %s", link_names[i].c_str());
    }

    const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform(link_names.back());
    
    tf::poseEigenToMsg(end_effector_state, response.end_effector_pose);
    return true;

}