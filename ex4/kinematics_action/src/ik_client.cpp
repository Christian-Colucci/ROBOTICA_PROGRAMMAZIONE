
#include <ros/ros.h>
#include <kinematics_action_msgs/GetIKSolutionAction.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <eigen_conversions/eigen_msg.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <condition_variable>

void DoneCallback(const actionlib::SimpleClientGoalState & state,const kinematics_action_msgs::GetIKSolutionResultConstPtr & result);
void ActiveCallback();
void FeedbackCallback(const kinematics_action_msgs::GetIKSolutionFeedbackConstPtr & feedback);
void SerializeSolution(std::ostringstream & ss,moveit_msgs::RobotState current_selection);


std::mutex mutex;
std::condition_variable result_handled;

int main(int argc, char**argv){

    ros::init(argc, argv, "ik_action_client_node");
    ros::NodeHandle nh;

    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<kinematics_action_msgs::GetIKSolutionAction> action_client("compute_ik_custom_action", true);

    action_client.waitForServer();
    
    kinematics_action_msgs::GetIKSolutionGoal goal;

    goal.end_effector_pose.position.x = 1.0;
    goal.end_effector_pose.position.y = 1.0;
    goal.end_effector_pose.position.z = 1.0;
    
    tf2::Quaternion quaternion;
    quaternion.setRPY(1.5708, 0.0, 0.0);

    goal.end_effector_pose.orientation = tf2::toMsg(quaternion);
    
    action_client.sendGoal(goal,&DoneCallback,&ActiveCallback,&FeedbackCallback);
    
    if(!action_client.waitForResult(ros::Duration(30.0)))
        ROS_ERROR("The IK solver did not complete in the allotted time");    

    /// serve per non far terminare il client che terminerà solo quando avrà pubblicato tutti i valori delle soluzioni sul topic joint_state_publisher    
    
    std::unique_lock<std::mutex> lock(mutex);
    result_handled.wait(lock);
    
    ros::shutdown();
    return 0;

}

void DoneCallback(const actionlib::SimpleClientGoalState & state,const kinematics_action_msgs::GetIKSolutionResultConstPtr & result){
    std::ostringstream ss;
    moveit_msgs::RobotState current_solution;
    int n_joints;

    if(state == actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED)
    {
        int num_solutions=result->robot_state.size();

        ss << "Goal achieved. " << state.getText() << std::endl;
        
        for (int i =0; i<num_solutions; i++){
            current_solution = result->robot_state[i];
            ss<<"Solution " << i+1 << " : [";
            SerializeSolution(ss,current_solution);    
            ss << std::endl;
        }

        ROS_INFO_STREAM(ss.str());

        ros::NodeHandle nh;

        // Instantiate the joint state publisher publishing on the joint_states topic
        ros::Publisher joint_state_publisher = nh.advertise<sensor_msgs::JointState>("joint_state_publisher", 1);

        // Load the robot model
        robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
        
        // Get the robot kinematic model
        robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

        // Get the planning group
        const robot_state::JointModelGroup * joint_model_group = kinematic_model->getJointModelGroup("fanuc");

        sensor_msgs::JointState joint_state_msg;
        joint_state_msg.name = joint_model_group->getVariableNames();

        ROS_INFO("Publishing solutions...");

        ros::Duration sleep_time(2.0);

        for(int i=0; i < num_solutions; i++)
        {
            sleep_time.sleep();

            joint_state_msg.position = result->robot_state[i].joint_state.position;
            joint_state_msg.header.stamp = ros::Time::now();

            joint_state_publisher.publish(joint_state_msg);
        }       

        ROS_INFO("All solutions published");        
    }
    else{
        ss << "Goal aborted. " << state.getText();
        ROS_INFO_STREAM(ss.str());
    }
    
    result_handled.notify_all();
}

void ActiveCallback()
{
    ROS_INFO("Inverse kinematics request sent to the IK resolution action server");
}

void FeedbackCallback(const kinematics_action_msgs::GetIKSolutionFeedbackConstPtr & feedback){
    std::ostringstream ss;
    ss << "Received IK Solution: [ ";

    SerializeSolution(ss,feedback->single_robot_state);

    ROS_INFO_STREAM(ss.str());

}

void SerializeSolution(std::ostringstream & ss,moveit_msgs::RobotState current_solution){
    int n_joints = current_solution.joint_state.position.size();
    
    for(int j=0; j<n_joints;j++){
        ss << current_solution.joint_state.position[j];
        if(j != n_joints - 1)
            ss << ", ";
        else{
            ss << " ]";
        }    
    }

}


