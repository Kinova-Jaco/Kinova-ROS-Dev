#ifndef KINOVA_JOINT_TRAJECTORY_CONTROLLER_H
#define KINOVA_JOINT_TRAJECTORY_CONTROLLER_H


#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>
#include "kinova_ros_types.h"

namespace kinova
{

class JointTrajectoryController
{
public:
    JointTrajectoryController(ros::NodeHandle &n);
    ~JointTrajectoryController();


private:
    ros::NodeHandle nh_;

    ros::Subscriber sub_command_;
    ros::Publisher pub_joint_feedback_;


    trajectory_msgs::JointTrajectory joint_traj_;
    std::vector<trajectory_msgs::JointTrajectoryPoint> joint_traj_points_;
//    trajectory_msgs::JointTrajectoryPoint joint_traj_point_;

    sensor_msgs::JointState current_joint_state_;

    TrajectoryPoint kinova_traj_point_;

    void commandCB(const trajectory_msgs::JointTrajectoryConstPtr &traj_msg);

};






}


#endif // KINOVA_JOINT_TRAJECTORY_CONTROLLER_H
