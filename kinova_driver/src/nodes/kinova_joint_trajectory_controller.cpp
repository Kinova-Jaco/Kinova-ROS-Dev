#include <actionlib/server/simple_action_server.h>


#include <kinova_driver/kinova_joint_trajectory_controller.h>


namespace kinova
{

JointTrajectoryController::JointTrajectoryController(ros::NodeHandle& n):
    nh_(n)
{
    ros::NodeHandle pn("~");

    sub_command_ = nh_.subscribe("/trajectory_controller/command", 1, &JointTrajectoryController::commandCB, this);

    pub_joint_feedback_ = nh_.advertise<control_msgs::FollowJointTrajectoryActionFeedback>("/trajectory_controller/state", 1);


}

JointTrajectoryController::~JointTrajectoryController()
{
    sub_command_.shutdown();
    pub_joint_feedback_.shutdown();

}




void JointTrajectoryController::commandCB(const trajectory_msgs::JointTrajectoryConstPtr &traj_msg)
{

}





}

int main(int argv, char** argc)
{
    ros::init(argv, argc, "kinova_joint_trajectory_controller");
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    kinova::JointTrajectoryController jtc(node);

    ros::spin();
    return 0;
}
