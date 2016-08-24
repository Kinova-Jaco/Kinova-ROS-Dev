/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit/move_group_interface/move_group.h>
#include <geometric_shapes/solid_primitive_dims.h>

#include <ros/console.h>

static const std::string ROBOT_DESCRIPTION="robot_description";

void pick(moveit::planning_interface::MoveGroup &group)
{
    std::vector<moveit_msgs::Grasp> grasps;

      geometry_msgs::PoseStamped p;
      p.header.frame_id = "root";
      p.pose.position.x = 0.32;
      p.pose.position.y = -0.3;
      p.pose.position.z = 0.5;
      p.pose.orientation.x = 0;
      p.pose.orientation.y = 0;
      p.pose.orientation.z = 0;
      p.pose.orientation.w = 1;
      moveit_msgs::Grasp g;
      g.grasp_pose = p;

      g.pre_grasp_approach.direction.vector.x = 1.0;
      g.pre_grasp_approach.direction.header.frame_id = "j2n6s300_end_effector";
      g.pre_grasp_approach.min_distance = 0.2;
      g.pre_grasp_approach.desired_distance = 0.4;

      g.post_grasp_retreat.direction.header.frame_id = "root";
      g.post_grasp_retreat.direction.vector.z = 1.0;
      g.post_grasp_retreat.min_distance = 0.1;
      g.post_grasp_retreat.desired_distance = 0.25;

      g.pre_grasp_posture.joint_names.resize(1, "j2n6s300_joint_finger_1");
      g.pre_grasp_posture.points.resize(1);
      g.pre_grasp_posture.points[0].positions.resize(1);
      g.pre_grasp_posture.points[0].positions[0] = 1;

      g.grasp_posture.joint_names.resize(1, "j2n6s300_joint_finger_1");
      g.grasp_posture.points.resize(1);
      g.grasp_posture.points[0].positions.resize(1);
      g.grasp_posture.points[0].positions[0] = 0;

      grasps.push_back(g);
      group.setSupportSurfaceName("table");
      group.pick("part", grasps);



    /*

    std::vector<moveit_msgs::Grasp> grasps;

    // pose of gripper when grasping
    geometry_msgs::PoseStamped p;
    //  p.header.frame_id = "root";
    //  p.pose.position.x = 0.5;
    //  p.pose.position.y = 0.0;
    //  p.pose.position.z = 0.13/1.0;
    //  p.pose.orientation.x = -0.495605587959;
    //  p.pose.orientation.y = -0.531222403049;
    //  p.pose.orientation.z = -0.575461030006;
    //  p.pose.orientation.w = -0.375529646873;

    p.header.stamp = ros::Time::now();
    p.header.frame_id = "root";
    p.pose.position.x = 0.5020;
    p.pose.position.y = -0.3910;
    p.pose.position.z = 0.4870;
    p.pose.orientation.x = 0.650062918663;
    p.pose.orientation.y = 0.319907665253;
    p.pose.orientation.z = 0.422783970833;
    p.pose.orientation.w = 0.544362962246;

    moveit_msgs::Grasp g;
    g.id = "grasp_part";
    g.grasp_pose = p;

    g.pre_grasp_approach.direction.header.stamp = ros::Time::now();
    g.pre_grasp_approach.direction.header.frame_id = "j2n6s300_end_effector";
    g.pre_grasp_approach.direction.vector.x = 0.0;
    g.pre_grasp_approach.direction.vector.y = 0.0;
    g.pre_grasp_approach.direction.vector.z = 1.0;
    g.pre_grasp_approach.min_distance = 0.01;
    g.pre_grasp_approach.desired_distance = 0.05;

    g.post_grasp_retreat.direction.header.stamp = ros::Time::now();
    g.post_grasp_retreat.direction.header.frame_id = "j2n6s300_end_effector";
    g.post_grasp_retreat.direction.vector.x = 0.0;
    g.post_grasp_retreat.direction.vector.y = 0.0;
    g.post_grasp_retreat.direction.vector.z = -1.0;
    g.post_grasp_retreat.min_distance = 0.0;
    g.post_grasp_retreat.desired_distance = 0.0;

    g.pre_grasp_posture.header.stamp = ros::Time::now();
    g.pre_grasp_posture.joint_names.resize(1, "j2n6s300_joint_finger_1");
    g.pre_grasp_posture.points.resize(1);
    g.pre_grasp_posture.points[0].positions.resize(1);
    g.pre_grasp_posture.points[0].positions[0] = 0;
    g.pre_grasp_posture.points[0].time_from_start = ros::Duration(120.0);

    g.grasp_posture.header.stamp = ros::Time::now();
    g.grasp_posture.joint_names.resize(1, "j2n6s300_joint_finger_1");
    g.grasp_posture.points.resize(1);
    g.grasp_posture.points[0].positions.resize(1);
    g.grasp_posture.points[0].positions[0] = 1.4;
    g.grasp_posture.points[0].time_from_start = ros::Duration(130.0);

    g.allowed_touch_objects.resize(1);
    g.allowed_touch_objects[0] = "part";


    grasps.push_back(g);
    //  group.setSupportSurfaceName("table");
    ROS_DEBUG_STREAM("" << __PRETTY_FUNCTION__ << ", line: " << __LINE__);
    group.setNumPlanningAttempts(5);
    group.allowReplanning(true);
    bool group_pick = group.pick("part", grasps);
    ROS_DEBUG_STREAM("" << __PRETTY_FUNCTION__ << ", line: " << __LINE__ << std::endl << "group_pick is: " << (group_pick == 1 ? "sccessful" : "failed") );
    */
}

void place(moveit::planning_interface::MoveGroup &group)
{
    std::vector<moveit_msgs::PlaceLocation> loc;

    geometry_msgs::PoseStamped p;
    p.header.frame_id = "root";
    p.pose.position.x = 0.7;
    p.pose.position.y = 0.0;
    p.pose.position.z = 0.5;
    p.pose.orientation.x = 0;
    p.pose.orientation.y = 0;
    p.pose.orientation.z = 0;
    p.pose.orientation.w = 1;
    moveit_msgs::PlaceLocation g;
    g.place_pose = p;

    g.pre_place_approach.direction.vector.z = -1.0;
    g.post_place_retreat.direction.vector.x = -1.0;
    g.post_place_retreat.direction.header.frame_id = "root";
    g.pre_place_approach.direction.header.frame_id = "r_wrist_roll_link";
    g.pre_place_approach.min_distance = 0.1;
    g.pre_place_approach.desired_distance = 0.2;
    g.post_place_retreat.min_distance = 0.1;
    g.post_place_retreat.desired_distance = 0.25;

    g.post_place_posture.joint_names.resize(1, "r_gripper_joint");
    g.post_place_posture.points.resize(1);
    g.post_place_posture.points[0].positions.resize(1);
    g.post_place_posture.points[0].positions[0] = 1;

    loc.push_back(g);
    group.setSupportSurfaceName("table");


    // add path constraints
    moveit_msgs::Constraints constr;
    constr.orientation_constraints.resize(1);
    moveit_msgs::OrientationConstraint &ocm = constr.orientation_constraints[0];
    ocm.link_name = "r_wrist_roll_link";
    ocm.header.frame_id = p.header.frame_id;
    ocm.orientation.x = 0.0;
    ocm.orientation.y = 0.0;
    ocm.orientation.z = 0.0;
    ocm.orientation.w = 1.0;
    ocm.absolute_x_axis_tolerance = 0.2;
    ocm.absolute_y_axis_tolerance = 0.2;
    ocm.absolute_z_axis_tolerance = M_PI;
    ocm.weight = 1.0;
    //  group.setPathConstraints(constr);
    group.setPlannerId("RRTConnectkConfigDefault");

    group.place("part", loc);
}

int main(int argc, char **argv)
{
    ros::init (argc, argv, "my_pick_place");
    ros::AsyncSpinner spinner(1);
    spinner.start();

        std::string pause;

    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    ros::NodeHandle nh;
    ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
    ros::Publisher pub_aco = nh.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 10);

    ros::WallDuration(1.0).sleep();

    moveit::planning_interface::MoveGroup group("arm");
    group.setPlanningTime(360.0);

    ros::WallDuration(1.0).sleep();

    // send robot to home position
    group.setNamedTarget("Home");
    group.move();
    ros::WallDuration(1.0).sleep();
    ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": send robot to home position");

    moveit_msgs::CollisionObject co;
    co.header.stamp = ros::Time::now();
    co.header.frame_id = "root";

//    ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ": no publishing ");
//    std::cin >> pause;

    // remove pole
      co.id = "pole";
      co.operation = moveit_msgs::CollisionObject::REMOVE;
      pub_co.publish(co);
//      ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ": remove pole ");
//      std::cin >> pause;

      // add pole
      co.operation = moveit_msgs::CollisionObject::ADD;
      co.primitives.resize(1);
      co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
      co.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
      co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.3;
      co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.1;
      co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 1.0;
      co.primitive_poses.resize(1);
      co.primitive_poses[0].position.x = 0.7;
      co.primitive_poses[0].position.y = 0.0; // -0.4
      co.primitive_poses[0].position.z = 0.85;
      co.primitive_poses[0].orientation.w = 1.0;
      pub_co.publish(co);
//      ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ": ADD pole ");
//      std::cin >> pause;


      // remove table
      co.id = "table";
      co.operation = moveit_msgs::CollisionObject::REMOVE;
      pub_co.publish(co);
//      ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ": remove table ");
//      std::cin >> pause;

      // add table
      co.operation = moveit_msgs::CollisionObject::ADD;
      co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.5;
      co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 1.5;
      co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.35;
      co.primitive_poses[0].position.x = 0.7;
      co.primitive_poses[0].position.y = -0.2;
      co.primitive_poses[0].position.z = 0.175;
      pub_co.publish(co);
//      ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ": ADD table ");
//      std::cin >> pause;

      co.id = "part";
      co.operation = moveit_msgs::CollisionObject::REMOVE;
      pub_co.publish(co);
//      ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ": remove part in co ");
//      std::cin >> pause;

      moveit_msgs::AttachedCollisionObject aco;
      aco.object = co;
      pub_aco.publish(aco);
//      ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ": remove part in aco ");
//      std::cin >> pause;


      co.operation = moveit_msgs::CollisionObject::ADD;
      co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.15;
      co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.1;
      co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.3;

      co.primitive_poses[0].position.x = 0.6;
      co.primitive_poses[0].position.y = -0.3; // -0.7
      co.primitive_poses[0].position.z = 0.5;
      pub_co.publish(co);

//      ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ": add part in co ");
//      std::cin >> pause;



    // try pick and place
    ROS_WARN_STREAM("going to pick, and press any key to continue");
    std::cin >> pause;

//    // wait a bit for ros things to initialize
//    ros::WallDuration(1.0).sleep();
//    ROS_DEBUG_STREAM("" << __PRETTY_FUNCTION__ << ", line: " << __LINE__);
    pick(group);
//    ROS_DEBUG_STREAM("" << __PRETTY_FUNCTION__ << ", line: " << __LINE__);
//    ros::WallDuration(1.0).sleep();

    //  place(group);

//    ros::waitForShutdown();
    ros::WallDuration(1.0).sleep();
    ros::shutdown();
    return 0;
}
