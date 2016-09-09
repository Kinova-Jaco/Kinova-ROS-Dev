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

    // pick process
    result_ = false;
    result_ = my_pick(group, gripper_group);

}


PickPlace::~PickPlace()
{
    //
}


void PickPlace::build_workscene()
{
    std::string pause;
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
          ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ": ADD table ");
          std::cin >> pause;

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

          ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ": add part in co_ ");
          std::cin >> pause;
}

geometry_msgs::PoseStamped define_grasp_pose()
{
    geometry_msgs::PoseStamped grasp_pose;
    grasp_pose.header.frame_id = "root";

    grasp_pose.pose.position.x = 0.416036397219;
    grasp_pose.pose.position.y = -0.285465538502;
    grasp_pose.pose.position.z = 0.507134079933;
    grasp_pose.pose.orientation.x = 0.572861850262;
    grasp_pose.pose.orientation.y = 0.402622461319;
    grasp_pose.pose.orientation.z = 0.51812005043;
    grasp_pose.pose.orientation.w = 0.491198539734;

    return grasp_pose;
}

geometry_msgs::PoseStamped generate_pregrasp_pose()
{
    geometry_msgs::PoseStamped pregrasp_pose;
    pregrasp_pose.header.frame_id = "root";

    pregrasp_pose.pose.position.x = 0.416036397219;
    pregrasp_pose.pose.position.y = -0.285465538502;
    pregrasp_pose.pose.position.z = 0.507134079933;
    pregrasp_pose.pose.orientation.x = 0.572861850262;
    pregrasp_pose.pose.orientation.y = 0.402622461319;
    pregrasp_pose.pose.orientation.z = 0.51812005043;
    pregrasp_pose.pose.orientation.w = 0.491198539734;

    return pregrasp_pose;
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
