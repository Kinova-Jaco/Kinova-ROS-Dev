#include <iostream>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <kinova_driver/kinova_ros_types.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "initial_pose");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    /* This sleep is ONLY to allow Rviz to come up */
    sleep(10.0);


    tf::Pose Home;
    Home.setOrigin(tf::Vector3(0.212322831154, -0.257197618484, 0.509646713734));
    Home.setRotation(kinova::EulerXYZ2Quaternion(1.63771402836,1.11316478252, 0.134094119072));

    tf::Pose random_pose;
    random_pose.setOrigin(tf::Vector3(0.54882, -0.30854,  0.65841));
    random_pose.setRotation(tf::Quaternion(0.68463, -0.22436, 0.68808, 0.086576));


    moveit::planning_interface::MoveGroup group("arm");
    moveit::planning_interface::PlanningSceneInterface scene;

    group.setGoalTolerance(0.001);

    // getPlanningFrame() is: /root
    ROS_INFO_STREAM("getPlanningFrame() is: " << group.getPlanningFrame());

    std::vector<std::string> names = group.getActiveJoints();
    for (int i=0; i<names.size(); i++)
    {
        ROS_INFO_STREAM("Active joints: " << names[i]);
    }

    ROS_WARN("---------------- Before planning goal -------------");


    ROS_INFO_STREAM("getCurrentPose() is: " << group.getCurrentPose("j2n6s300_end_effector").pose );
    ROS_INFO_STREAM("printStatePositions() is: ");
    group.getCurrentState()->printStatePositions();


    ROS_WARN("---------------- After planing goal -------------");

    geometry_msgs::Pose goal;
    tf::poseTFToMsg(random_pose, goal);
    group.setPoseTarget(goal);


    geometry_msgs::Pose get_goal;
    get_goal = group.getPoseTarget().pose;
    ROS_INFO_STREAM("goal pose is: " << get_goal);

    robot_state::RobotState robot_state_goal = *group.getCurrentState();

    robot_state_goal.setFromIK(robot_state_goal.getJointModelGroup(group.getName()), goal);
// // another way to set joint value target by Cartesian Pose target.
//    const double pose_value[7] = {goal.position.x, goal.position.y, goal.position.z, goal.orientation.x, goal.orientation.y, goal.orientation.z, goal.orientation.w};
//    robot_state_goal.setVariablePositions(pose_value);

    group.setJointValueTarget(robot_state_goal);

    ROS_WARN("get_joint.setFromIK: ");
    robot_state_goal.printStatePositions();


    // execution the planed motion //
    moveit::planning_interface::MoveGroup::Plan my_plan;
    if(group.plan(my_plan))
    {
        ROS_WARN("Success in group.plan(my_plan)");
        group.move();
    }

    ROS_WARN("----------- After execution (group.move) ----------");

    ROS_INFO_STREAM("; getCurrentPose() is: " << group.getCurrentPose("j2n6s300_end_effector") );

    group.getCurrentState()->printStatePositions();


    group.getJointValueTarget().printStateInfo();
    group.getJointValueTarget().printStatePositions();

    return 0;
}
