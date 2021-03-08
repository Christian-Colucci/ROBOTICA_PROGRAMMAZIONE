#include <ros/ros.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <eigen_conversions/eigen_msg.h>
#include <kinematics_action_msgs/GetIKSolutionAction.h>
#include <actionlib/server/simple_action_server.h>
#include <angles/angles.h>

class IKAction
    {
    protected:
    
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<kinematics_action_msgs::GetIKSolutionAction> action_server_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    std::string action_name_;
    // create messages that are used to published feedback/result
    kinematics_action_msgs::GetIKSolutionFeedback feedback_;
    kinematics_action_msgs::GetIKSolutionResult result_;
   
    public:
   
    IKAction(std::string name) :
        action_server_(nh_, name, boost::bind(&IKAction::compute_IK, this, _1), false),
        action_name_(name)
    {
    action_server_.start();
    }

void compute_IK(const kinematics_action_msgs::GetIKSolutionGoalConstPtr &goal){
    bool success = true;

    ROS_INFO("Start Inverse Kinematics");

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
    robot_state::RobotState robot_state(kinematic_model);

    robot_state::RobotStatePtr kinematic_state= std::make_shared<robot_state::RobotState>(robot_state);
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("fanuc");
    const kinematics::KinematicsBaseConstPtr solver = joint_model_group->getSolverInstance();

    int ik_call_counter=0;
    std::vector<double> joint_values;
    std::vector<std::vector<double>> check_new_solution;

    //ROS_INFO("ciaooo");
    while(ik_call_counter<1000 && ros::ok())
    {
        //ROS_INFO("ciaooo");
        const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
        std::vector<double> seed_state = generateSeedState_(kinematic_model);
        std::vector<double> solution;
        moveit_msgs::MoveItErrorCodes error_code;

        solver->getPositionIK(goal->end_effector_pose, seed_state, joint_values, error_code);

        if (action_server_.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("%s: Preempted", action_name_.c_str());
            // set the action state to preempted
            action_server_.setPreempted();
            success = false;
            break;
        }


        if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
            normalizeJointPositions_(joint_model_group,joint_values);
            if(NewSolution(joint_model_group,joint_values,check_new_solution))
            {
                check_new_solution.push_back(joint_values);
                robot_state.setVariablePositions(joint_values);
                //kinematic_state->setJointGroupPositions(joint_model_group,joint_values);
                moveit::core::robotStateToRobotStateMsg(robot_state, feedback_.single_robot_state);

                action_server_.publishFeedback(feedback_);

                result_.robot_state.push_back(feedback_.single_robot_state);

                for (std::size_t i = 0; i < joint_names.size(); ++i)
                    {
                        ROS_INFO("\n");
                        ROS_INFO("New Solution: ");
                        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
                    }
            }
        } 
        ik_call_counter++;

    }

    if(success){
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        // set the action state to succeeded
        action_server_.setSucceeded(result_);
    }

}

/// Controllo se nuova soluzione attraverso la distanza angolare perchè possiamo avere valori distanti tra di loro ma in realtà rappresentano 
/// posizioni nel piano rotazionale molto vicine tra di loro. Ecco perchè non possiamo fare la differenza tra i valori degli angoli.

bool NewSolution(const robot_state::JointModelGroup* joint_model_group,std::vector<double> & joint_values, std::vector<std::vector<double>> & check_new_solution) 
{   
    for(int i=0; i < check_new_solution.size(); i++)
    {
        bool are_solutions_equal = true;

        for(int j=0; j < check_new_solution[i].size() && are_solutions_equal; j++)
        {
            double diff;

            if(joint_model_group->getActiveJointModels()[j]->getType() == robot_model::JointModel::REVOLUTE)
            {
                diff = angles::shortest_angular_distance(check_new_solution[i][j], joint_values[j]);
            }
            else
            {
                diff = check_new_solution[i][j] - joint_values[j];
            }

            if(std::fabs(diff) > 1e-3)
                are_solutions_equal = false;
        }

        if(are_solutions_equal)
            return false;
    }

    return true;
}

/// Serve per generare seed diversi, quindi punti iniziali diversi, per l'algoritmo di ricerca su cu si basa il solver numerico che utilizziamo (KDL)
/// Questo serve per ottenere soluzioni di cinematica inversa diverse con la stessa posa.
std::vector<double> generateSeedState_(robot_model::RobotModelPtr & kinematic_model) const
{
    std::vector<double> seed_state;

    std::vector<std::string> joint_names = kinematic_model->getVariableNames();

    for(int i=0; i < joint_names.size(); i++)
    {
        double lb = kinematic_model->getURDF()->getJoint(joint_names[i])->limits->lower;
        double ub = kinematic_model->getURDF()->getJoint(joint_names[i])->limits->upper;
        double span = ub-lb;
        
        seed_state.push_back((double)std::rand()/RAND_MAX * span + lb);
    }

    return seed_state;
}

/// La normalizzazione serve per portare angoli in intervalli diversi in un unico intervallo in modo da poter essere dopo confrontati nella funzione NewSolution.
void normalizeJointPositions_(const robot_state::JointModelGroup* joint_model_group,std::vector<double> & joint_values) const
{
    for(int i=0; i < joint_values.size(); i++)
    {
        if (joint_model_group->getActiveJointModels()[i]->getType() == robot_model::JointModel::REVOLUTE)
        {
            joint_values[i] = angles::normalize_angle(joint_values[i]);
        }
    }
}

};


int main(int argc, char**argv)
{
    ros::init(argc, argv, "ik_action_server_node");
    ros::NodeHandle nh;
    IKAction ik_action("compute_ik_custom_action");

    ROS_INFO("Started FK service ");
    ros::spin();
    ros::shutdown();
    return 0;
}