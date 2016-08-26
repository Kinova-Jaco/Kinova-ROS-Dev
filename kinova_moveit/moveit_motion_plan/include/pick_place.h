#ifndef PICK_PLACE_H
#define PICK_PLACE_H

#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <geometric_shapes/solid_primitive_dims.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>
namespace kinova
{
    class PickPlace
    {
    public:
        PickPlace(ros::NodeHandle &nh, moveit::planning_interface::MoveGroup &group, moveit::planning_interface::MoveGroup &gripper_group);
        ~PickPlace();



    private:
        ros::NodeHandle nh_;


        // work scene
        moveit_msgs::CollisionObject co_;
        moveit_msgs::AttachedCollisionObject aco_;
        moveit_msgs::PlanningScene planning_scene_;

        ros::Publisher pub_co_;
        ros::Publisher pub_aco_;
        ros::Publisher pub_planning_scene_diff_;

        //
        std::vector<std::string> joint_names_;
        std::vector<double> joint_values_;

        // use Kinova Inverse Kinematic model to generate joint value, then setJointTarget().
        bool use_KinovaInK;

        // check some process if success.
        bool result_;

        void build_workscene();
        geometry_msgs::PoseStamped define_grasp_pose();
        geometry_msgs::PoseStamped generate_pregrasp_pose(geometry_msgs::Pose grasp_pose, double dist, double azimuth, double polar, double rot_gripper_z);
        geometry_msgs::PoseStamped generate_postgrasp_pose();
        bool my_pick(moveit::planning_interface::MoveGroup &group, moveit::planning_interface::MoveGroup &gripper_group);
        bool my_place(moveit::planning_interface::MoveGroup &group, moveit::planning_interface::MoveGroup &gripper_group);
        void getInvK(geometry_msgs::Pose &eef_pose, std::vector<double> &joint_value);

    };
}


#endif // PICK_PLACE_H
