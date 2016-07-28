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
    traj_joint_points_ = traj_msg->points;

    kinova_comm_.stopAPI();

    KinovaAngles kinova_angle;
    for (size_t i = 0; i<traj_joint_points_.size(); i++)
    {
        kinova_angle.Actuator1 = traj_joint_points_[i].positions[0];
        kinova_angle.Actuator2 = traj_joint_points_[i].positions[1];
        kinova_angle.Actuator3 = traj_joint_points_[i].positions[2];
        kinova_angle.Actuator4 = traj_joint_points_[i].positions[3];
        kinova_angle.Actuator5 = traj_joint_points_[i].positions[4];
        kinova_angle.Actuator6 = traj_joint_points_[i].positions[5];

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


    trajectory_msgs::JointTrajectoryPoint feedback_desired;
    trajectory_msgs::JointTrajectoryPoint feedback_actual;
    trajectory_msgs::JointTrajectoryPoint feedback_error;


    control_msgs::FollowJointTrajectoryFeedback joint_trajectory_feedback;
    joint_trajectory_feedback.header.frame_id = traj_frame_id_;
    joint_trajectory_feedback.header.stamp = ros::Time::now();
    joint_trajectory_feedback.joint_names = traj_joint_names_;
    joint_trajectory_feedback.desired = feedback_desired;
    joint_trajectory_feedback.actual = feedback_actual;
    joint_trajectory_feedback.error = feedback_error;

    pub_joint_feedback_.publish(joint_trajectory_feedback);


}
