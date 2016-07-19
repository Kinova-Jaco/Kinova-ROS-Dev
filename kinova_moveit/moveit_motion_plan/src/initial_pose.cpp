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
    double retract_joint_value[6] = {-1.57079622, 2.61799388,  0.4712389 , -1.6057028 ,  0.08726646, 1.74532925};

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




    ROS_WARN("---------------- Initial state -------------");

    moveit::planning_interface::MoveGroup group("arm");
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model_ptr=robot_model_loader.getModel();
    robot_state::RobotStatePtr robot_state_ptr(group.getCurrentState());
    robot_state_ptr->setToDefaultValues(robot_state_ptr->getJointModelGroup(group.getName()), "home");
    robot_state_ptr->update();

    group.setGoalJointTolerance(0.1*M_PI/180);
    group.setGoalPositionTolerance(0.001);
    group.setGoalOrientationTolerance(0.1*M_PI/180);
    // getPlanningFrame() is: /root
    ROS_INFO_STREAM("getPlanningFrame() is: " << group.getPlanningFrame());

    ROS_INFO_STREAM("getCurrentPose() is: " << group.getCurrentPose("j2n6s300_end_effector").pose );
    ROS_INFO_STREAM("printStatePositions() is: ");
    group.getCurrentState()->printStatePositions();



    ROS_WARN("---------------- Setting to Home state -------------");

    robot_state::RobotState home_state = *group.getCurrentState();
    home_state.setVariablePositions(home_joint_value);
    ROS_INFO("Home state is defined: ");
    home_state.printStatePositions();

    group.setJointValueTarget(home_state);
    group.move();
    sleep(5);

    ROS_INFO("home joint reached:");
    group.getCurrentState()->printStatePositions();
    ROS_INFO_STREAM("home pose reached: " << group.getCurrentPose().pose);



    ROS_WARN("---------------- Moving to goal -------------");

//    robot_state::RobotState goal_state = *group.getCurrentState();
//    goal_state.setFromIK(goal_state.getJointModelGroup(group.getName()), Retract,10,1);
//    goal_state.update();
//    group.setJointValueTarget(goal_state);

    group.setPoseTarget(Retract);

    ROS_INFO("Goal planed: ");
    group.getJointValueTarget().printStatePositions();

    moveit::planning_interface::MoveGroup::Plan my_plan;
    if(group.plan(my_plan))
    {
        sleep(5);
        ROS_INFO("Success in group.plan(my_plan)");
        group.execute(my_plan);
        sleep(5);
    }

    ROS_INFO_STREAM("Goal joint reached: ");
    group.getCurrentState()->printStatePositions();
    ROS_INFO_STREAM("Goal pose reached: " << group.getCurrentPose("j2n6s300_end_effector").pose );

    return 0;
}
