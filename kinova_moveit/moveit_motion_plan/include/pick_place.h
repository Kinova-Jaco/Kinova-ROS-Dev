#ifndef PICK_PLACE_H
#define PICK_PLACE_H

#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit/move_group_interface/move_group.h>
#include <geometric_shapes/solid_primitive_dims.h>

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

        ros::Publisher pub_co_;
        ros::Publisher pub_aco_;

        //
        std::vector<std::string> joint_names_;
        std::vector<double> joint_values_;

        // use Kinova Inverse Kinematic model to generate joint value, then setJointTarget().
        bool use_KinovaInK;

        // check some process if success.
        bool result_;

        void build_workscene();
        geometry_msgs::PoseStamped define_grasp_pose();
        geometry_msgs::PoseStamped generate_pregrasp_pose();
        geometry_msgs::PoseStamped generate_postgrasp_pose();
        bool my_pick(moveit::planning_interface::MoveGroup &group, moveit::planning_interface::MoveGroup &gripper_group);
        bool my_place(moveit::planning_interface::MoveGroup &group, moveit::planning_interface::MoveGroup &gripper_group);
        void getInvK(geometry_msgs::Pose &eef_pose, std::vector<double> &joint_value);

    };
}


#endif // PICK_PLACE_H
