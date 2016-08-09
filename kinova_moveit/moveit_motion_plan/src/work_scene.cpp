#include <ros/ros.h>

#include <geometry_msgs/Pose.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>



int main(int argc, char** argv)
{
    ros::init(argc, argv, "work_scene");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh;

    // Display debug information in teminal
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }


    // Define publisher to update work scene
    ros::Publisher pub_work_scene = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    while(pub_work_scene.getNumSubscribers() < 1)
    {
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
    }

    // Define table plane
    shape_msgs::SolidPrimitive table;
    table.type = table.BOX;
    table.dimensions.resize(3);
    table.dimensions[0] = 1.6;
    table.dimensions[1] = 0.8;
    table.dimensions[2] = 0.03;

    // Define table position
    geometry_msgs::Pose table_pose;
    table_pose.orientation.w = 1.0;
    table_pose.position.x = table.dimensions[0]/2.0 - 0.15;
    table_pose.position.y = table.dimensions[1]/2.0 - 0.08;
    table_pose.position.z =  -table.dimensions[2]/2.0;

    // Define collision objects
    moveit_msgs::CollisionObject collision_objects;
    collision_objects.id = "table";
    collision_objects.header.frame_id = "root";
    collision_objects.primitives.push_back(table);
    collision_objects.primitive_poses.push_back(table_pose);

    collision_objects.operation = collision_objects.ADD;

    // Add collision objects to environment
    ROS_INFO("Adding the collision objects to the work scene.");
    moveit_msgs::PlanningScene work_scene;
    work_scene.world.collision_objects.push_back(collision_objects);
    work_scene.is_diff = true;
    pub_work_scene.publish(work_scene);
    ros::Duration(1).sleep();


//    Define attached objects
//    moveit_msgs::AttachedCollisionObject attached_object;
//    attached_object.link_name = "j2n6s300_end_effector";
//    attached_object.object.header.frame_id = "j2n6s300_end_effector";
//    attached_object.object.id = "cylinder";


    ros::shutdown();


    return 0;
}
