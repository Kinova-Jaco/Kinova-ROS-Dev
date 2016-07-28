#ifndef KINOVA_JOINT_TRAJECTORY_CONTROLLER_H
#define KINOVA_JOINT_TRAJECTORY_CONTROLLER_H


#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Duration.h>

#include "kinova_ros_types.h"
#include "kinova_comm.h"
#include "kinova_api.h"

namespace kinova
{

class JointTrajectoryController
{
public:
    JointTrajectoryController(kinova::KinovaComm &kinova_comm, ros::NodeHandle &n);
    ~JointTrajectoryController();


private:
    ros::NodeHandle nh_;
    KinovaComm kinova_comm_;

    ros::Subscriber sub_command_;
    ros::Publisher pub_joint_feedback_;


//    trajectory_msgs::JointTrajectory joint_traj_;
//    trajectory_msgs::JointTrajectoryPoint joint_traj_point_;
    std::string traj_frame_id_;
    std::vector<std::string> traj_joint_names_;
    std::vector<trajectory_msgs::JointTrajectoryPoint> traj_joint_points_;
    std_msgs::Duration time_from_start_;



    sensor_msgs::JointState current_joint_state_;

    TrajectoryPoint kinova_traj_point_;


    // call back function when receive a trajectory command
    void commandCB(const trajectory_msgs::JointTrajectoryConstPtr &traj_msg);
    // reflash the robot state and publish the joint state
    void update();

};






}


#endif // KINOVA_JOINT_TRAJECTORY_CONTROLLER_H
