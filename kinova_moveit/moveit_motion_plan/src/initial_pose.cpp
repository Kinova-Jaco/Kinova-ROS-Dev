#include <iostream>
#include <map>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <kinova_driver/kinova_ros_types.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

typedef std::map<std::string, double> state_map;

geometry_msgs::Pose to_geometry_pose(tf::Pose tf_pose)
{
    geometry_msgs::Pose geometry_pose;
    geometry_pose.position.x = tf_pose.getOrigin().x();
    geometry_pose.position.y = tf_pose.getOrigin().y();
    geometry_pose.position.z = tf_pose.getOrigin().z();

    tf::Quaternion q = tf_pose.getRotation();
    geometry_pose.orientation.x = q.x();
    geometry_pose.orientation.y = q.y();
    geometry_pose.orientation.z = q.z();
    geometry_pose.orientation.w = q.w();

    return geometry_pose;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "initial_pose");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    /* This sleep is ONLY to allow Rviz to come up */
    sleep(10.0);

    // Retract (rest pose)
    tf::Pose tf_retract;
    tf_retract.setOrigin(tf::Vector3(0.028, -0.1714, 0.2995));
    tf_retract.setRotation(tf::Quaternion(0.68801, -0.5464, 0.32995, 0.34528));
    geometry_msgs::Pose Retract = to_geometry_pose(tf_retract);

    // Home (ready pose)
    tf::Pose tf_home;
    tf_home.setOrigin(tf::Vector3(0.212322831154, -0.257197618484, 0.509646713734));
//    tf_home.setRotation(kinova::EulerXYZ2Quaternion(1.63771402836,1.11316478252, 0.134094119072));
    tf_home.setRotation(tf::Quaternion(0.68463, -0.22436, 0.68808, 0.086576));
    geometry_msgs::Pose Home = to_geometry_pose(tf_home);
    double home_joint_value[6] = {-1.47550822, 2.92151711,  1.00484714, -2.08487422,  1.44703677, 1.31851771};

    tf::Pose tf_random_pose;
    tf_random_pose.setOrigin(tf::Vector3(0.54882, -0.30854,  0.65841));
    tf_random_pose.setRotation(tf::Quaternion(0.68463, -0.22436, 0.68808, 0.086576));
    geometry_msgs::Pose Random = to_geometry_pose(tf_random_pose);


    moveit::planning_interface::MoveGroup group("arm");
    moveit::planning_interface::PlanningSceneInterface scene;

    group.setGoalJointTolerance(0.1*M_PI/180);
    group.setGoalPositionTolerance(0.001);
    group.setGoalOrientationTolerance(0.1*M_PI/180);

    // getPlanningFrame() is: /root
    ROS_INFO_STREAM("getPlanningFrame() is: " << group.getPlanningFrame());

    std::vector<std::string> names = group.getActiveJoints();
    for (int i=0; i<names.size(); i++)
    {
        ROS_INFO_STREAM("Active joints: " << names[i]);
    }

    ROS_WARN("---------------- Initial state -------------");

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model_ptr=robot_model_loader.getModel();
    robot_state::RobotStatePtr robot_state_ptr(group.getCurrentState());
    robot_state_ptr->setToDefaultValues(robot_state_ptr->getJointModelGroup(group.getName()), "home");

    ROS_WARN("DefaultPlannedID is: %s.", group.getDefaultPlannerId().c_str());


    ROS_INFO_STREAM("getCurrentPose() is: " << group.getCurrentPose("j2n6s300_end_effector").pose );
    ROS_INFO_STREAM("printStatePositions() is: ");
    group.getCurrentState()->printStatePositions();


    ROS_WARN("---------------- Set to Home state -------------");

    robot_state::RobotState home_state = *group.getCurrentState();
    // joint value of Home pose (ready pose)
//    double home_joint_value[6] = {-1.47550822, 2.92151711,  1.00484714, -2.08487422,  1.44703677, 1.31851771};
//    home_state.setVariablePositions(home_joint_value);
    ROS_WARN_STREAM("Home is: " << Home);

    home_state.setFromIK(home_state.getJointModelGroup(group.getName()), Home);
    ROS_WARN("Home state is: ");
    home_state.printStatePositions();

    group.setJointValueTarget(home_state);
    group.move();
    sleep(5);
    std::vector<double> current_joint = group.getCurrentJointValues();
    ROS_INFO("home joint is: %f, %f, %f, %f, %f, %f.", current_joint[0],current_joint[1], current_joint[2], current_joint[3], current_joint[4],current_joint[5]);
    ROS_INFO_STREAM("home pose is: " << group.getCurrentPose().pose);


    ROS_WARN("---------------- After planing goal -------------");

    robot_state::RobotState goal_state = *group.getCurrentState();

    // setFromIK() does not work here?
    /*
    geometry_msgs::Pose goal;
    tf::poseTFToMsg(tf_retract, goal);
    group.setPoseTarget(goal);
    goal_state.setFromIK(goal_state.getJointModelGroup(group.getName()), goal);
    */

    // joint value of Retract pose (rest pose)
    double goal_joint_value[6] = {-1.57079622, 2.61799388,  0.4712389 , -1.6057028 ,  0.08726646, 1.74532925};
    goal_state.setVariablePositions(goal_joint_value);

    group.setJointValueTarget(goal_state);

    ROS_INFO("goal in Joint space: ");
    goal_state.printStatePositions();


    ROS_WARN("----------- Moving to goal (group.move) ----------");
    moveit::planning_interface::MoveGroup::Plan my_plan;
    if(group.plan(my_plan))
    {
        ROS_WARN("Success in group.plan(my_plan)");
        group.move();
        sleep(5);
    }

    ROS_WARN("----------- After execution ----------");

    ROS_INFO_STREAM("GoalJointTolerance is: " << group.getGoalJointTolerance() );
    ROS_INFO_STREAM("printStatePositions() is: ");
    group.getCurrentState()->printStatePositions();
//    ROS_INFO_STREAM("After execution pose is: " << group.getCurrentPose("j2n6s300_end_effector").pose );

    return 0;
}
