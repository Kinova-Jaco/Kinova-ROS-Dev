#include <pick_place.h>
#include <ros/console.h>

using namespace kinova;

PickPlace::PickPlace(ros::NodeHandle &nh, moveit::planning_interface::MoveGroup &group, moveit::planning_interface::MoveGroup &gripper_group):
    nh_(nh)
{
    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    ros::NodeHandle pn("~");

    pub_co_ = nh_.advertise<moveit_msgs::CollisionObject>("/collision_object", 10);
    pub_aco_ = nh_.advertise<moveit_msgs::AttachedCollisionObject>("/attached_collision_object", 10);
    pub_planning_scene_diff_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

    int arm_joint_num = 6;
    joint_names_.resize(arm_joint_num);
    joint_values_.resize(joint_names_.size());
    for (uint i = 0; i<joint_names_.size(); i++)
    {
        joint_names_[i] = "j2n6s300_joint_" + boost::lexical_cast<std::string>(i+1);
    }


    // send robot to home position
    group.setNamedTarget("Home");
    group.move();
    ros::WallDuration(1.0).sleep();
    ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": send robot to home position");

    // add collision objects
    build_workscene();

    ros::WallDuration(1.0).sleep();

//    // pick process
//    result_ = false;
//    result_ = my_pick(group, gripper_group);

    // check pregrasp pose
    define_grasp_pose();

    ros::WallDuration(1.0).sleep();

    generate_pregrasp_pose(0.01, 0.0, 0.0, 0.0);

}


PickPlace::~PickPlace()
{
    //
}


void PickPlace::build_workscene()
{
    std::string pause;
    co_.header.frame_id = "root";
    co_.header.stamp = ros::Time::now();

    // remove pole
    co_.id = "pole";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);
    //      ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ": remove pole ");
    //      std::cin >> pause;

    // add pole
    co_.operation = moveit_msgs::CollisionObject::ADD;
    co_.primitives.resize(1);
    co_.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    co_.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.3;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.1;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 1.0;
    co_.primitive_poses.resize(1);
    co_.primitive_poses[0].position.x = 0.7;
    co_.primitive_poses[0].position.y = 0.0; // -0.4
    co_.primitive_poses[0].position.z = 0.85;
    co_.primitive_poses[0].orientation.w = 1.0;
    pub_co_.publish(co_);
    planning_scene_.world.collision_objects.push_back(co_);
    planning_scene_.is_diff = true;
    pub_planning_scene_diff_.publish(planning_scene_);
    ros::WallDuration(0.1).sleep();
    //      ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ": ADD pole ");
    //      std::cin >> pause;


    // remove table
    co_.id = "table";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);
    //      ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ": remove table ");
    //      std::cin >> pause;

    // add table
    co_.operation = moveit_msgs::CollisionObject::ADD;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.5;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 1.5;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.35;
    co_.primitive_poses[0].position.x = 0.7;
    co_.primitive_poses[0].position.y = -0.2;
    co_.primitive_poses[0].position.z = 0.175;
    pub_co_.publish(co_);
    planning_scene_.world.collision_objects.push_back(co_);
    planning_scene_.is_diff = true;
    pub_planning_scene_diff_.publish(planning_scene_);
    ros::WallDuration(0.1).sleep();
//          ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ": ADD table ");
//          std::cin >> pause;

    co_.id = "part";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);
    //      ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ": remove part in co_ ");
    //      std::cin >> pause;

    aco_.object = co_;
    pub_aco_.publish(aco_);
    //      ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ": remove part in aco_ ");
    //      std::cin >> pause;


    co_.operation = moveit_msgs::CollisionObject::ADD;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.15;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.1;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.3;

    co_.primitive_poses[0].position.x = 0.6;
    co_.primitive_poses[0].position.y = -0.3; // -0.7
    co_.primitive_poses[0].position.z = 0.5;
    pub_co_.publish(co_);
    planning_scene_.world.collision_objects.push_back(co_);
    planning_scene_.is_diff = true;
    pub_planning_scene_diff_.publish(planning_scene_);
    ros::WallDuration(0.1).sleep();

//          ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ": add part in co_ ");
//          std::cin >> pause;
}


void PickPlace::define_grasp_pose()
{
    grasp_pose_.header.frame_id = "root";
    grasp_pose_.header.stamp  = ros::Time::now();

    grasp_pose_.pose.position.x = 0.416036397219;
    grasp_pose_.pose.position.y = -0.285465538502;
    grasp_pose_.pose.position.z = 0.507134079933;
    grasp_pose_.pose.orientation.x = 0.572861850262;
    grasp_pose_.pose.orientation.y = 0.402622461319;
    grasp_pose_.pose.orientation.z = 0.51812005043;
    grasp_pose_.pose.orientation.w = 0.491198539734;
}

/**
 * @brief generate_pregrasp_pose
 * @param grasp_pose the pose where gripper close the fingers (expect to grasp the object)
 * @param dist radius, distance of pregrasp_pose to grasp_pose
 * @param azimuth an angle measured from the x-axis in the xy-plane in spherical coordinates, denoted theta (0<= theta < 2pi ).
 * @param polar also named zenith, colatitude, denoted phi (0<=phi<=pi). It is the angle from the positive z-axis to the vector.  phi= pi/2 - delta where delta is the latitude.
 * @param rot_gripper_z rotation along the z axis of the gripper reference frame (last joint rotation)
 * @return pregrasp_pose
 */
void PickPlace::generate_pregrasp_pose(double dist, double azimuth, double polar, double rot_gripper_z)
{
    pregrasp_pose_.header.frame_id = "root";

    // computer pregrasp position w.r.t. location of grasp_pose in spherical coordinate. Orientation is w.r.t. fixed world (root) reference frame.
    double delta_x = dist * cos(azimuth) * sin(polar);
    double delta_y = dist * sin(azimuth) * sin(polar);
    double delta_z = dist * cos(polar);

    // computer the orientation of gripper w.r.t. fixed world (root) reference frame. The gripper (z axis) should point(open) to the grasp_pose.
    tf::Quaternion q;
    tf::Matrix3x3 rot_matrix, rot_matrix_gripper_z;

    // pointing to grasp pose
    rot_matrix.setEulerZYX(azimuth, polar, M_PI);
    // rotate by rot_gripper_z
    rot_matrix_gripper_z.setEulerZYX(rot_gripper_z, 0, 0);
    // rot_matrix = rot_matrix * rot_matrix_gripper_z;
    rot_matrix *= rot_matrix_gripper_z;
    rot_matrix.getRotation(q);

    pregrasp_pose_.pose.position.x = grasp_pose_.pose.position.x + delta_x;
    pregrasp_pose_.pose.position.y = grasp_pose_.pose.position.y + delta_y;
    pregrasp_pose_.pose.position.z = grasp_pose_.pose.position.z + delta_z;
    pregrasp_pose_.pose.orientation.x = q.x();
    pregrasp_pose_.pose.orientation.y = q.y();
    pregrasp_pose_.pose.orientation.z = q.z();
    pregrasp_pose_.pose.orientation.w = q.w();

    ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE: " << __LINE__ << ": " << "pregrasp_pose_: x " << pregrasp_pose_.pose.position.x  << ", y " << pregrasp_pose_.pose.position.y  << ", z " << pregrasp_pose_.pose.position.z  << ", qx " << pregrasp_pose_.pose.orientation.x  << ", qy " << pregrasp_pose_.pose.orientation.y  << ", qz " << pregrasp_pose_.pose.orientation.z  << ", qw " << pregrasp_pose_.pose.orientation.w );

}


bool PickPlace::my_pick(moveit::planning_interface::MoveGroup &group, moveit::planning_interface::MoveGroup &gripper_group)
{
    std::vector<double> grasp_pose_joint;
    grasp_pose_joint.resize(joint_names_.size());
//    getInvK(grasp_pose, grasp_pose_joint);
    grasp_pose_joint[0] = -120.716 *M_PI/180.0;
    grasp_pose_joint[1] = 192.656 *M_PI/180.0;
    grasp_pose_joint[2] = 78.033 *M_PI/180.0;
    grasp_pose_joint[3] = -126.818 *M_PI/180.0;
    grasp_pose_joint[4] = 64.909 *M_PI/180.0;
    grasp_pose_joint[5] = 91.227 *M_PI/180.0;

    group.setJointValueTarget(grasp_pose_joint);

    moveit::planning_interface::MoveGroup::Plan my_plan;

    bool replan = true;
    while (replan == true && ros::ok())
    {
        result_ = group.plan(my_plan);
        std::cout << "plan is sccess? " << (result_? "yes" : "no") << std::endl;
        std::cout << "please input 1 to replan, or 0 to execute the plan: ";
        std::cin >> replan;
    }

    if (result_ == true)
    {
        group.execute(my_plan);
    }

}


void PickPlace::getInvK(geometry_msgs::Pose &eef_pose, std::vector<double> &joint_value)
{

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pick_place_demo");
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // MoveIt
    moveit::planning_interface::MoveGroup group("arm");
    moveit::planning_interface::MoveGroup gripper_group("gripper");

    kinova::PickPlace pick_place(node, group, gripper_group);

    ros::spin();
    return 0;
}
