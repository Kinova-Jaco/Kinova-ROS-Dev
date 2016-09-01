#include <pick_place.h>
#include <ros/console.h>

#include <tf_conversions/tf_eigen.h>

using namespace kinova;

tf::Quaternion EulerZYZ_to_Quaternion(double tz1, double ty, double tz2)
{
    tf::Quaternion q;
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

    rot.getRotation(q);
    return q;
}


PickPlace::PickPlace(ros::NodeHandle &nh):
    nh_(nh)
{
    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    ros::NodeHandle pn("~");

    sub_joint_ = nh_.subscribe<sensor_msgs::JointState>("/j2n6s300_driver/out/joint_state", 1, &PickPlace::get_current_state, this);
    sub_pose_ = nh_.subscribe<geometry_msgs::PoseStamped>("/j2n6s300_driver/out/tool_pose", 1, &PickPlace::get_current_pose, this);

    // Before we can load the planner, we need two objects, a RobotModel and a PlanningScene.
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model_ = robot_model_loader.getModel();

    // construct a `PlanningScene` that maintains the state of the world (including the robot).
    planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

//    //  every time need retrive current robot state, do the following.
//    robot_state::RobotState& robot_state = planning_scene_->getCurrentStateNonConst();
//    const robot_state::JointModelGroup *joint_model_group = robot_state.getJointModelGroup("arm");

    group_ = new moveit::planning_interface::MoveGroup("arm");
    gripper_group_ = new moveit::planning_interface::MoveGroup("gripper");

    group_->setEndEffectorLink("j2n6s300_end_effector");

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
    ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": LINE: " << __LINE__ << ": ");
    ROS_DEBUG_STREAM("send robot to home position");

//    ROS_INFO("group_ home joint reached:");
//    group_->getCurrentState()->printStatePositions();
//    ROS_INFO_STREAM("group_ home pose reached: " << group_->getCurrentPose().pose);

    // add collision objects
    build_workscene();

    ros::WallDuration(1.0).sleep();

    // pick process
    result_ = false;
    result_ = my_pick();

}


PickPlace::~PickPlace()
{
    //
}


void PickPlace::get_current_state(const sensor_msgs::JointStateConstPtr &msg)
{
    boost::mutex::scoped_lock lock(mutex_state_);
    current_state_ = *msg;
}

void PickPlace::get_current_pose(const geometry_msgs::PoseStampedConstPtr &msg)
{
    boost::mutex::scoped_lock lock(mutex_pose_);
    current_pose_ = *msg;
}


void PickPlace::build_workscene()
{
    std::string pause;
    co_.header.frame_id = "root";
    co_.header.stamp = ros::Time::now();



    // remove table
    co_.id = "table";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);
    //      ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ": remove table ");
    //      std::cin >> pause;

    // add table
    co_.primitives.resize(1);
    co_.primitive_poses.resize(1);
    co_.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    co_.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    co_.operation = moveit_msgs::CollisionObject::ADD;

    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 1.6;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.8;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.03;
    co_.primitive_poses[0].position.x = 1.6/2.0 - 0.1;
    co_.primitive_poses[0].position.y = 0.8/2.0 - 0.1;
    co_.primitive_poses[0].position.z = -0.03/2.0;
    pub_co_.publish(co_);
    planning_scene_msg_.world.collision_objects.push_back(co_);
    planning_scene_msg_.is_diff = true;
    pub_planning_scene_diff_.publish(planning_scene_msg_);
    ros::WallDuration(0.1).sleep();
//          ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ": ADD table ");
//          std::cin >> pause;



    co_.id = "coca_can";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);
    //      ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ": remove part in co_ ");
    //      std::cin >> pause;

    aco_.object = co_;
//    aco_.link_name = "j2n6s300_end_effector";
//    const robot_model::JointModelGroup* gripper_group = robot_model_->getJointModelGroup("gripper");
//    const std::vector<std::string>& touch_links = gripper_group->getLinkModelNames();
//    for (int i = 0; i<touch_links.size(); i++)
//    {
//        aco_.touch_links.push_back(touch_links[i]);
//    }
//    aco_.touch_links.push_back("j2n6s300_joint_6");
    pub_aco_.publish(aco_);
    //      ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ": remove part in aco_ ");
    //      std::cin >> pause;

    co_.primitives.resize(1);
    co_.primitive_poses.resize(1);
    co_.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
    co_.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
    co_.operation = moveit_msgs::CollisionObject::ADD;

    double coca_h = 0.13;
    double coca_r = 0.036;
    double coca_pos_x = 0.5;
    double coca_pos_y = 0.5;
    double coca_pos_z = coca_h/2.0;

    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = coca_h;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = coca_r;
    co_.primitive_poses[0].position.x = coca_pos_x;
    co_.primitive_poses[0].position.y = coca_pos_y;
    co_.primitive_poses[0].position.z = coca_pos_z;
    pub_co_.publish(co_);
    planning_scene_msg_.world.collision_objects.push_back(co_);
    planning_scene_msg_.is_diff = true;
    pub_planning_scene_diff_.publish(planning_scene_msg_);
    ros::WallDuration(0.1).sleep();
//          ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ": add part in co_ ");
//          std::cin >> pause;

    // remove pole
    co_.id = "pole";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);
    //      ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ": remove pole ");
    //      std::cin >> pause;


    // add obstacle between robot and object
    co_.primitives.resize(1);
    co_.primitive_poses.resize(1);
    co_.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    co_.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    co_.operation = moveit_msgs::CollisionObject::ADD;

    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = coca_r *1.5;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = coca_r *1.5;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = coca_h *1.5;
//    co_.primitive_poses[0].position.x = coca_pos_x/2.0;
//    co_.primitive_poses[0].position.y = coca_pos_y/2.0;
    co_.primitive_poses[0].position.x = coca_pos_x;
        co_.primitive_poses[0].position.y = 0.1;
    co_.primitive_poses[0].position.z = coca_h*1.5/2.0;
    co_.primitive_poses[0].orientation.w = 1.0;
    pub_co_.publish(co_);
    planning_scene_msg_.world.collision_objects.push_back(co_);
    planning_scene_msg_.is_diff = true;
    pub_planning_scene_diff_.publish(planning_scene_msg_);
    ros::WallDuration(0.1).sleep();
    //      ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ": ADD pole ");
    //      std::cin >> pause;


}

void PickPlace::check_collision()
{
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    collision_request.contacts = true;
    collision_request.max_contacts = 1000;

    collision_result.clear();
    planning_scene_->checkCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 1: Current state is "
                    << (collision_result.collision ? "in" : "not in")
                    << " collision");

    collision_request.group_name = "arm";
    collision_result.clear();
    planning_scene_->checkCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 3: Current state is "
                    << (collision_result.collision ? "in" : "not in")
                    << " collision");

    // check contact
    planning_scene_->checkCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 4: Current state is "
                    << (collision_result.collision ? "in" : "not in")
                    << " collision");
    collision_detection::CollisionResult::ContactMap::const_iterator it;
    for(it = collision_result.contacts.begin();
        it != collision_result.contacts.end();
        ++it)
    {
        ROS_INFO("Contact between: %s and %s",
                 it->first.first.c_str(),
                 it->first.second.c_str());
    }

    // allowed collision matrix
    collision_detection::AllowedCollisionMatrix acm = planning_scene_->getAllowedCollisionMatrix();
    robot_state::RobotState copied_state = planning_scene_->getCurrentState();

    collision_detection::CollisionResult::ContactMap::const_iterator it2;
    for(it2 = collision_result.contacts.begin();
        it2 != collision_result.contacts.end();
        ++it2)
    {
        acm.setEntry(it2->first.first, it2->first.second, true);
    }
    collision_result.clear();
    planning_scene_->checkCollision(collision_request, collision_result, copied_state, acm);
    ROS_INFO_STREAM("Test 5: Current state is "
                    << (collision_result.collision ? "in" : "not in")
                    << " collision");
}

void PickPlace::define_grasp_pose()
{
    grasp_pose_.header.frame_id = "root";
    grasp_pose_.header.stamp  = ros::Time::now();

    // Euler_ZYZ (-M_PI/4, M_PI/2, M_PI/2)
    grasp_pose_.pose.position.x = 0.5;
    grasp_pose_.pose.position.y = 0.5;
    grasp_pose_.pose.position.z = 0.1;

    tf::Quaternion q = EulerZYZ_to_Quaternion(M_PI/4, M_PI/2, M_PI/2);
    grasp_pose_.pose.orientation.x = q.x();
    grasp_pose_.pose.orientation.y = q.y();
    grasp_pose_.pose.orientation.z = q.z();
    grasp_pose_.pose.orientation.w = q.w();

//    robot_state::RobotState start_state(*group.getCurrentState());
//    robot_state::RobotState& robot_state = planning_scene->getCurrentStateNonConst();
    robot_state::RobotState& grasp_state = planning_scene_->getCurrentStateNonConst();
    const robot_state::JointModelGroup *joint_model_group =
                    grasp_state.getJointModelGroup(group_->getName());
    grasp_state.setFromIK(joint_model_group, grasp_pose_.pose);
//    group.setStartState(start_state);

//    ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": LINE: " << __LINE__ << ": " << "grasp_state.printStatePositions();" );
//    grasp_state.printStatePositions();

//    group_->setPoseTarget(grasp_pose_);
    group_->setJointValueTarget(grasp_pose_.pose);
    ros::WallDuration(0.1).sleep();
    grasp_state = group_->getJointValueTarget();
    ros::WallDuration(0.1).sleep();
//    ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": LINE: " << __LINE__ << ": " << "grasp_state.printStatePositions();" );
//    grasp_state.printStatePositions();

    // Now we will plan to the earlier pose target from the new
    // start state that we have just created.
//    group.setPoseTarget(target_pose1);
//    success = group.plan(my_plan);

        grasp_joint_.resize(joint_names_.size());
    //    getInvK(grasp_pose, grasp_joint_);
        grasp_joint_[0] = 155.4 *M_PI/180.0;
        grasp_joint_[1] = 256.5 *M_PI/180.0;
        grasp_joint_[2] = 127.8 *M_PI/180.0;
        grasp_joint_[3] = 240.8 *M_PI/180.0;
        grasp_joint_[4] = 82.7 *M_PI/180.0;
        grasp_joint_[5] = 97.9 *M_PI/180.0;

//        pregrasp_joint_.resize(joint_names_.size());
//    //    getInvK(pregrasp_pose, pregrasp_joint_);
//        pregrasp_joint_[0] = 145.4 *M_PI/180.0;
//        pregrasp_joint_[1] = 253.7 *M_PI/180.0;
//        pregrasp_joint_[2] = 67.0 *M_PI/180.0;
//        pregrasp_joint_[3] = 151.0 *M_PI/180.0;
//        pregrasp_joint_[4] = 118.5 *M_PI/180.0;
//        pregrasp_joint_[5] = 141.7 *M_PI/180.0;


        pregrasp_joint_.resize(joint_names_.size());
        pregrasp_joint_[0] = 229.4 *M_PI/180.0;
        pregrasp_joint_[1] = 255.0 *M_PI/180.0;
        pregrasp_joint_[2] = 105.4 *M_PI/180.0;
        pregrasp_joint_[3] = 237.0 *M_PI/180.0;
        pregrasp_joint_[4] = 112.5 *M_PI/180.0;
        pregrasp_joint_[5] = 112.2 *M_PI/180.0;

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
    tf::Quaternion q = EulerZYZ_to_Quaternion(azimuth, polar, rot_gripper_z);

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

void PickPlace::setup_constrain(geometry_msgs::Pose target_pose)
{
    // setup constrains
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = "j2n6s300_end_effector";
    ocm.header.frame_id = "root";
    ocm.orientation = target_pose.orientation;
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;
    ocm.weight = 0.5;


    /* Define position constrain box based on current pose and target pose. */
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);

    // group_->getCurrentPose() does not work.
//    current_pose_ = group_->getCurrentPose();
    geometry_msgs::Pose current_pose;
    { // scope for mutex update
        boost::mutex::scoped_lock lock_pose(mutex_pose_);
        current_pose = current_pose_.pose;
//        ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": LINE: " << __LINE__ << ": " << "current_pose_: x " << current_pose_.pose.position.x  << ", y " << current_pose_.pose.position.y  << ", z " << current_pose_.pose.position.z  << ", qx " << current_pose_.pose.orientation.x  << ", qy " << current_pose_.pose.orientation.y  << ", qz " << current_pose_.pose.orientation.z  << ", qw " << current_pose_.pose.orientation.w );
    }

    double constrain_box_scale = 2.0;
    primitive.dimensions[0] = constrain_box_scale * std::abs(target_pose.position.x - current_pose.position.x);
    primitive.dimensions[1] = constrain_box_scale * std::abs(target_pose.position.y - current_pose.position.y);
    primitive.dimensions[2] = constrain_box_scale * std::abs(target_pose.position.z - current_pose.position.z);

    /* A pose for the box (specified relative to frame_id) */
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    // place between start point and goal point.
    box_pose.position.x = (target_pose.position.x + current_pose.position.x)/2.0;
    box_pose.position.y = (target_pose.position.y + current_pose.position.y)/2.0;
    box_pose.position.z = (target_pose.position.z + current_pose.position.z)/2.0;

    moveit_msgs::PositionConstraint pcm;
    pcm.link_name = "j2n6s300_end_effector";
    pcm.header.frame_id = "root";
    pcm.constraint_region.primitives.push_back(primitive);
    pcm.constraint_region.primitive_poses.push_back(box_pose);
    pcm.weight = 0.5;

    moveit_msgs::Constraints grasp_constrains;
    grasp_constrains.orientation_constraints.push_back(ocm);
    group_->setPathConstraints(grasp_constrains);


//    // The bellowing code is just for visulizing the box and check.
//    // Disable this part after checking.
//    co_.id = "check_constrain";
//    co_.operation = moveit_msgs::CollisionObject::REMOVE;
//    pub_co_.publish(co_);

//    co_.operation = moveit_msgs::CollisionObject::ADD;
//    co_.primitives.push_back(primitive);
//    co_.primitive_poses.push_back(box_pose);
//    pub_co_.publish(co_);
//    planning_scene_msg_.world.collision_objects.push_back(co_);
//    planning_scene_msg_.is_diff = true;
//    pub_planning_scene_diff_.publish(planning_scene_msg_);
//    ros::WallDuration(0.1).sleep();
}


bool PickPlace::my_pick()
{


    { // scope for mutex update
        boost::mutex::scoped_lock lock_state(mutex_state_);
//        ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": LINE: " << __LINE__ << ": " );
//        for (int i = 0; i< 9; i++)
//        {
//            ROS_DEBUG_STREAM(current_state_.name[i] << ": " << current_state_.position[i] << ", " );
//        }
    }


    define_grasp_pose();

    ros::WallDuration(0.1).sleep();

    // generate_pregrasp_pose(double dist, double azimuth, double polar, double rot_gripper_z)
//    pregrasp_pose_ = generate_gripper_align_pose(grasp_pose_, 0.1, M_PI/4, M_PI/2, M_PI/2);
//    postgrasp_pose_ = pregrasp_pose_;


//    group_->setPoseTarget(grasp_pose_);
//    setup_constrain(pregrasp_pose_.pose);

    ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": LINE: " << __LINE__ << ": " );
    for (int i = 0; i< 6; i++)
    {
        ROS_DEBUG_STREAM("joint " << i+1 << " is: " << pregrasp_joint_[i] *180/M_PI << ", " );
    }
    group_->setJointValueTarget(pregrasp_joint_);
    ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": LINE: " << __LINE__ << "give anything to move to pregrasp " );
    std::string test1;
    std::cin >> test1;

    group_->move();

    std::string test2;
    std::cin >> test2;

    group_->setJointValueTarget(grasp_joint_);

    moveit::planning_interface::MoveGroup::Plan my_plan;

    bool replan = true;
    while (replan == true && ros::ok())
    {
        result_ = group_->plan(my_plan);
        check_collision();
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
