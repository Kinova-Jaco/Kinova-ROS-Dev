#include <pick_place.h>
#include <ros/console.h>

using namespace kinova;

tf::Matrix3x3 setEulerZYZ(double tz1, double ty, double tz2)
{
    tf::Matrix3x3 rot;
    tf::Matrix3x3 rot_temp;
    rot.setIdentity();

    rot_temp.setEulerZYX(tz1, 0.0, 0.0);
    rot *= rot_temp;
    rot_temp.setEulerZYX(0.0, ty, 0.0);
    rot *= rot_temp;
    rot_temp.setEulerZYX(tz2, 0.0, 0.0);
    rot *= rot_temp;

//    ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": LINE: " << __LINE__ << ": " << std::endl << "rot_x 1st row : " << rot.getRow(0).getX() << ", " << rot.getRow(0).getY() << ", " << rot.getRow(0).getZ() << ", "  << std::endl << "rot_x 2nd row : " << rot.getRow(1).getX() << ", " << rot.getRow(1).getY() << ", " << rot.getRow(1).getZ() << ", "  << std::endl << "rot_x 3rd row : " << rot.getRow(2).getX() << ", " << rot.getRow(2).getY() << ", " << rot.getRow(2).getZ());
    return rot;
}


PickPlace::PickPlace(ros::NodeHandle &nh):
    nh_(nh)
{
    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    ros::NodeHandle pn("~");

//    sub_joint_ = nh_.subscribe<"", 1, &getCurrentJoint, this);

    group_ = new moveit::planning_interface::MoveGroup("arm");
    gripper_group_ = new moveit::planning_interface::MoveGroup("gripper");

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
    group_->setNamedTarget("Home");
//    group_->move();
    ros::WallDuration(1.0).sleep();
    ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": send robot to home position");

    // add collision objects
//    build_workscene();

    ros::WallDuration(1.0).sleep();

    // pick process
    result_ = false;
    result_ = my_pick();


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

    // Euler_ZYZ (-M_PI/4, M_PI/2, M_PI/2)
    grasp_pose_.pose.position.x = 0.5;
    grasp_pose_.pose.position.y = -0.5;
    grasp_pose_.pose.position.z = 0.05;
    grasp_pose_.pose.orientation.x = 0.653281;
    grasp_pose_.pose.orientation.y = 0.270598;
    grasp_pose_.pose.orientation.z = 0.270598;
    grasp_pose_.pose.orientation.w = 0.653281;
}


/**
 * @brief PickPlace::generate_gripper_align_pose
 * @param targetpose_msg pick/place pose (object location): where gripper close/open the fingers (grasp/release the object). Only position information is used.
 * @param dist distance of returned pose to targetpose
 * @param azimuth an angle measured from the x-axis in the xy-plane in spherical coordinates, denoted theta (0<= theta < 2pi ).
 * @param polar also named zenith, colatitude, denoted phi (0<=phi<=pi). It is the angle from the positive z-axis to the vector.  phi= pi/2 - delta where delta is the latitude.
 * @param rot_gripper_z rotation along the z axis of the gripper reference frame (last joint rotation)
 * @return a pose defined in a spherical coordinates where origin is located at the target pose. Normally it is a pre_grasp/post_realease pose, where gripper axis (last joint axis) is pointing to the object (target_pose).
 */
geometry_msgs::PoseStamped PickPlace::generate_gripper_align_pose(geometry_msgs::PoseStamped targetpose_msg, double dist, double azimuth, double polar, double rot_gripper_z)
{
    geometry_msgs::PoseStamped pose_msg;

    pose_msg.header.frame_id = "root";

    // computer pregrasp position w.r.t. location of grasp_pose in spherical coordinate. Orientation is w.r.t. fixed world (root) reference frame.
    double delta_x = -dist * cos(azimuth) * sin(polar);
    double delta_y = -dist * sin(azimuth) * sin(polar);
    double delta_z = -dist * cos(polar);

    // computer the orientation of gripper w.r.t. fixed world (root) reference frame. The gripper (z axis) should point(open) to the grasp_pose.
    tf::Quaternion q;
    tf::Matrix3x3 rot_matrix;

    // pointing to grasp pose and rotate by rot_gripper_z
    rot_matrix = setEulerZYZ(azimuth, polar, rot_gripper_z);
    rot_matrix.getRotation(q);

    pose_msg.pose.position.x = targetpose_msg.pose.position.x + delta_x;
    pose_msg.pose.position.y = targetpose_msg.pose.position.y + delta_y;
    pose_msg.pose.position.z = targetpose_msg.pose.position.z + delta_z;
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": LINE: " << __LINE__ << ": " << "pose_msg: x " << pose_msg.pose.position.x  << ", y " << pose_msg.pose.position.y  << ", z " << pose_msg.pose.position.z  << ", qx " << pose_msg.pose.orientation.x  << ", qy " << pose_msg.pose.orientation.y  << ", qz " << pose_msg.pose.orientation.z  << ", qw " << pose_msg.pose.orientation.w );

    return pose_msg;

}


bool PickPlace::my_pick()
{

    define_grasp_pose();

    ros::WallDuration(1.0).sleep();

    // generate_pregrasp_pose(double dist, double azimuth, double polar, double rot_gripper_z)
    pregrasp_pose_ = generate_gripper_align_pose(grasp_pose_, 0.1, -M_PI/4, M_PI/2, M_PI/2);

    postgrasp_pose_ = pregrasp_pose_;

//    // joint space

//    std::vector<double> grasp_pose_joint;
//    grasp_pose_joint.resize(joint_names_.size());
////    getInvK(grasp_pose, grasp_pose_joint);
//    grasp_pose_joint[0] = -120.716 *M_PI/180.0;
//    grasp_pose_joint[1] = 192.656 *M_PI/180.0;
//    grasp_pose_joint[2] = 78.033 *M_PI/180.0;
//    grasp_pose_joint[3] = -126.818 *M_PI/180.0;
//    grasp_pose_joint[4] = 64.909 *M_PI/180.0;
//    grasp_pose_joint[5] = 91.227 *M_PI/180.0;

//    group_->setJointValueTarget(grasp_pose_joint);

    std::vector<geometry_msgs::PoseStamped> pose_seq;
    pose_seq.push_back(pregrasp_pose_);
    pose_seq.push_back(grasp_pose_);
    pose_seq.push_back(postgrasp_pose_);

//    group_->setPoseTargets(pose_seq);

    group_->setPoseTarget(grasp_pose_);


    // setup constrains
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = "j2n6s300_end_effector";
    ocm.header.frame_id = "root";
    ocm.orientation = grasp_pose_.pose.orientation;
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;
    ocm.weight = 1.0;


    /* Define position constrain box . */
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    // group_->getCurrentPose() does not work.
    current_pose_ = group_->getCurrentPose();
    double constrain_box_scale = 1.5;
    primitive.dimensions[0] = constrain_box_scale * std::abs(grasp_pose_.pose.position.x - current_pose_.pose.position.x);
    primitive.dimensions[1] = constrain_box_scale * std::abs(grasp_pose_.pose.position.y - current_pose_.pose.position.y);
    primitive.dimensions[2] = constrain_box_scale * std::abs(grasp_pose_.pose.position.z - current_pose_.pose.position.z);

    /* A pose for the box (specified relative to frame_id) */
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    // place between start point and goal point.
    box_pose.position.x = (grasp_pose_.pose.position.x + current_pose_.pose.position.x)/2.0;
    box_pose.position.y = (grasp_pose_.pose.position.y + current_pose_.pose.position.y)/2.0;
    box_pose.position.z = (grasp_pose_.pose.position.z + current_pose_.pose.position.z)/2.0;

    moveit_msgs::PositionConstraint pcm;
    pcm.link_name = "j2n6s300_end_effector";
    pcm.header.frame_id = "root";
    pcm.constraint_region.primitives.push_back(primitive);
    pcm.constraint_region.primitive_poses.push_back(box_pose);
    pcm.weight = 0.5;

    moveit_msgs::Constraints grasp_constrains;
    grasp_constrains.orientation_constraints.push_back(ocm);
    group_->setPathConstraints(grasp_constrains);

    moveit::planning_interface::MoveGroup::Plan my_plan;

    bool replan = true;
    while (replan == true && ros::ok())
    {
        result_ = group_->plan(my_plan);
        std::cout << "plan is sccess? " << (result_? "yes" : "no") << std::endl;
        std::cout << "please input 1 to replan, or 0 to execute the plan: ";
        std::cin >> replan;
    }

    if (result_ == true)
    {
        group_->execute(my_plan);
    }

    return true;
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

    kinova::PickPlace pick_place(node);

    ros::spin();
    return 0;
}
