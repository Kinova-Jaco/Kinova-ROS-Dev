#include <kinova_driver/kinova_joint_trajectory_controller.h>


using namespace kinova;

JointTrajectoryController::JointTrajectoryController(kinova::KinovaComm &kinova_comm, ros::NodeHandle& n):
    kinova_comm_(kinova_comm),
    nh_(n)
{
    ros::NodeHandle pn("~");

    sub_command_ = nh_.subscribe("/trajectory_controller/command", 1, &JointTrajectoryController::commandCB, this);

    pub_joint_feedback_ = nh_.advertise<control_msgs::FollowJointTrajectoryFeedback>("/trajectory_controller/state", 1);

}

JointTrajectoryController::~JointTrajectoryController()
{
    sub_command_.shutdown();
    pub_joint_feedback_.shutdown();

}



void JointTrajectoryController::commandCB(const trajectory_msgs::JointTrajectoryConstPtr &traj_msg)
{
    traj_frame_id_ = traj_msg->header.frame_id;
    traj_joint_names_ = traj_msg->joint_names;
    joint_traj_points_ = traj_msg->points;

    kinova_comm_.stopAPI();

    KinovaAngles kinova_angle;
    for (size_t i = 0; i<joint_traj_points_.size(); i++)
    {
        kinova_angle.Actuator1 = joint_traj_points_[i].positions[0];
        kinova_angle.Actuator2 = joint_traj_points_[i].positions[1];
        kinova_angle.Actuator3 = joint_traj_points_[i].positions[2];
        kinova_angle.Actuator4 = joint_traj_points_[i].positions[3];
        kinova_angle.Actuator5 = joint_traj_points_[i].positions[4];
        kinova_angle.Actuator6 = joint_traj_points_[i].positions[5];

        ROS_WARN_STREAM("add kinova_angle to trajectory list: ");
        kinova_comm_.printAngles(kinova_angle);

        // add joint_angles to Trajectory list
//        kinova_comm_.setJointAngles(kinova_angle, 0, false);

    }

    ros::Duration(1.0).sleep();

    kinova_comm_.startAPI();


}

void JointTrajectoryController::update()
{
    KinovaAngles current_joint_angles;
    KinovaAngles current_joint_velocity;
    KinovaAngles current_joint_effort;

    kinova_comm_.getJointAngles(current_joint_angles);
    kinova_comm_.getJointVelocities(current_joint_velocity);
    kinova_comm_.getJointTorques(current_joint_effort);


}
