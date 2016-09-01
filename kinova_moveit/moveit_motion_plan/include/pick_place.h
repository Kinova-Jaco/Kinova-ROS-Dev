#ifndef PICK_PLACE_H
#define PICK_PLACE_H

#include <ros/ros.h>
#include <kinova_driver/kinova_ros_types.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_pipeline/planning_pipeline.h>

#include <moveit/kinematic_constraints/utils.h>
#include <geometric_shapes/solid_primitive_dims.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/DisplayTrajectory.h>

namespace kinova
{


    class PickPlace
    {
    public:
        PickPlace(ros::NodeHandle &nh);
        ~PickPlace();



    private:
        ros::NodeHandle nh_;

        moveit::planning_interface::MoveGroup* group_;
        moveit::planning_interface::MoveGroup* gripper_group_;
        robot_model::RobotModelPtr robot_model_;
//        robot_state::RobotStatePtr robot_state_;

        planning_scene::PlanningScenePtr planning_scene_;
        planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

        // work scene
        moveit_msgs::CollisionObject co_;
        moveit_msgs::AttachedCollisionObject aco_;
        moveit_msgs::PlanningScene planning_scene_msg_;


        ros::Publisher pub_co_;
        ros::Publisher pub_aco_;
        ros::Publisher pub_planning_scene_diff_;
        ros::Subscriber sub_pose_;
        ros::Subscriber sub_joint_;

        //
        std::vector<std::string> joint_names_;
        std::vector<double> joint_values_;

        // use Kinova Inverse Kinematic model to generate joint value, then setJointTarget().
        bool use_KinovaInK_;

        // check some process if success.
        bool result_;

        // update current state and pose
        boost::mutex mutex_state_;
        boost::mutex mutex_pose_;
        sensor_msgs::JointState current_state_;
        geometry_msgs::PoseStamped current_pose_;


        // define pick_place joint value and pose
        std::vector<double> grasp_joint_;
        std::vector<double> pregrasp_joint_;
        std::vector<double> postgrasp_joint_;

        geometry_msgs::PoseStamped grasp_pose_;
        geometry_msgs::PoseStamped pregrasp_pose_;
        geometry_msgs::PoseStamped postgrasp_pose_;


        void build_workscene();
        void define_grasp_pose();
        geometry_msgs::PoseStamped generate_gripper_align_pose(geometry_msgs::PoseStamped targetpose_msg, double dist, double azimuth, double polar, double rot_gripper_z);
        void setup_constrain(geometry_msgs::Pose target_pose);

        bool my_pick();
        bool my_place();

        void get_current_state(const sensor_msgs::JointStateConstPtr &msg);
        void get_current_pose(const geometry_msgs::PoseStampedConstPtr &msg);
        // TODO: use Kinova inverse kinematic solution instead of from ROS.
        void getInvK(geometry_msgs::Pose &eef_pose, std::vector<double> &joint_value);

    };
}


#endif // PICK_PLACE_H
