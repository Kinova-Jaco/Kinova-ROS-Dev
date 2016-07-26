#include <joint_trajectory_action_server.h>

using namespace kinova;

JointTrajectoryActionController::JointTrajectoryActionController(ros::NodeHandle &n):
    nh_(n),
    has_active_goal_(false)
{
    action_server_follow_.reset(new FJTAS(nh_, "/j2n6s300/follow_joint_trajectory", boost::bind(&JointTrajectoryActionController::goalCBFollow, this, _1), boost::bind(&JointTrajectoryActionController::cancelCBFollow, this, _1), false));

    ros::NodeHandle pn("~");

    int arm_joint_num = 6;

    std::vector<std::string> joint_names_;
    joint_names_.resize(arm_joint_num);

    for (uint i = 0; i<joint_names_.size(); i++)
    {
        joint_names_[i] = "j2n6s300_joint_" + boost::lexical_cast<std::string>(i+1);
        std::cout << "joint names: " << joint_names_[i] << std::endl;
    }

    pn.param("constraints/goal_time", goal_time_constraint_, 0.0);
    // Gets the constraints for each joint.
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        std::string ns = std::string("constraints/") + joint_names_[i];
        double g, t;
        pn.param(ns + "/goal", g, -1.0);
        pn.param(ns + "/trajectory", t, -1.0);
        goal_constraints_[joint_names_[i]] = g;
        trajectory_constraints_[joint_names_[i]] = t;
    }

    pn.param("constraints/stopped_velocity_tolerance", stopped_velocity_tolerance_, 0.01);


    pub_controller_command_ = nh_.advertise<trajectory_msgs::JointTrajectory>("command", 1);
    sub_controller_state_ = nh_.subscribe("state", 1, &JointTrajectoryActionController::controllerStateCB, this);

    watchdog_timer_ = nh_.createTimer(ros::Duration(1.0), &JointTrajectoryActionController::watchdog, this);


    ros::Time started_waiting_for_controller = ros::Time::now();
//    while (ros::ok() && !last_controller_state_)
//    {
//        ros::spinOnce();
//        if (started_waiting_for_controller != ros::Time(0) &&
//                ros::Time::now() > started_waiting_for_controller + ros::Duration(30.0))
//        {
//            ROS_WARN("Waited for the controller for 30 seconds, but it never showed up.");
//            started_waiting_for_controller = ros::Time(0);
//        }
//        ros::Duration(0.1).sleep();
//    }


    ROS_WARN("Start Follow_Joint_Trajectory_Action server!");
    action_server_follow_->start();


}


JointTrajectoryActionController::~JointTrajectoryActionController()
{
    pub_controller_command_.shutdown();
    sub_controller_state_.shutdown();
    watchdog_timer_.stop();
}


static bool setsEqual(const std::vector<std::string> &a, const std::vector<std::string> &b)
{
    if (a.size() != b.size())
        return false;

    for (size_t i = 0; i < a.size(); ++i)
    {
        if (count(b.begin(), b.end(), a[i]) != 1)
            return false;
    }
    for (size_t i = 0; i < b.size(); ++i)
    {
        if (count(a.begin(), a.end(), b[i]) != 1)
            return false;
    }

    return true;
}


void JointTrajectoryActionController::watchdog(const ros::TimerEvent &e)
{
    ros::Time now = ros::Time::now();

    // Aborts the active goal if the controller does not appear to be active.
    if (has_active_goal_)
    {
        bool should_abort = false;
        if (!last_controller_state_)
        {
            should_abort = true;
            ROS_WARN("Aborting goal because we have never heard a controller state message.");
        }
        else if ((now - last_controller_state_->header.stamp) > ros::Duration(5.0))
        {
            should_abort = true;
            ROS_WARN("Aborting goal because we haven't heard from the controller in %.3lf seconds",
                     (now - last_controller_state_->header.stamp).toSec());
        }

        if (should_abort)
        {
            // Stops the controller.
            trajectory_msgs::JointTrajectory empty;
            empty.joint_names = joint_names_;
            pub_controller_command_.publish(empty);

            // Marks the current goal as aborted.
            active_goal_.setAborted();
            has_active_goal_ = false;
        }
    }
}


void JointTrajectoryActionController::goalCBFollow(FJTAS::GoalHandle gh)
{

    // Ensures that the joints in the goal match the joints we are commanding.
    if (!setsEqual(joint_names_, gh.getGoal()->trajectory.joint_names))
    {
        ROS_ERROR("Joints on incoming goal don't match our joints");
        gh.setRejected();
        return;
    }


    // Cancels the currently active goal.
    if (has_active_goal_)
    {
        // Stops the controller.
        trajectory_msgs::JointTrajectory empty;
        empty.joint_names = joint_names_;
        pub_controller_command_.publish(empty);

        // Marks the current goal as canceled.
        active_goal_.setCanceled();
        has_active_goal_ = false;
    }

    gh.setAccepted();
    active_goal_ = gh;
    has_active_goal_ = true;


    // Sends the trajectory along to the controller
    current_traj_ = active_goal_.getGoal()->trajectory;
    pub_controller_command_.publish(current_traj_);
}


void JointTrajectoryActionController::cancelCBFollow(FJTAS::GoalHandle gh)
{
    if (active_goal_ == gh)
    {
        // Stops the controller.
        trajectory_msgs::JointTrajectory empty;
        empty.joint_names = joint_names_;
        pub_controller_command_.publish(empty);

        // Marks the current goal as canceled.
        active_goal_.setCanceled();
        has_active_goal_ = false;
    }

}


void JointTrajectoryActionController::controllerStateCB(const control_msgs::FollowJointTrajectoryActionFeedbackConstPtr &msg)
{
    last_controller_state_ = msg;
    ros::Time now = ros::Time::now();

    if (!has_active_goal_)
        return;
    if (current_traj_.points.empty())
        return;
    if (now < current_traj_.header.stamp + current_traj_.points[0].time_from_start)
        return;

    if (!setsEqual(joint_names_, msg->feedback.joint_names))
    {
        ROS_ERROR("Joint names from the controller don't match our joint names.");
        return;
    }

    int last = current_traj_.points.size() - 1;
    ros::Time end_time = current_traj_.header.stamp + current_traj_.points[last].time_from_start;

    // Verifies that the controller has stayed within the trajectory constraints.

    if (now < end_time)
    {
        // Checks that the controller is inside the trajectory constraints.
        for (size_t i = 0; i < msg->feedback.joint_names.size(); ++i)
        {
            double abs_error = fabs(msg->feedback.error.positions[i]);
            double constraint = trajectory_constraints_[msg->feedback.joint_names[i]];
            if (constraint >= 0 && abs_error > constraint)
            {
                // Stops the controller.
                trajectory_msgs::JointTrajectory empty;
                empty.joint_names = joint_names_;
                pub_controller_command_.publish(empty);

                active_goal_.setAborted();
                has_active_goal_ = false;
                ROS_WARN("Aborting because we would up outside the trajectory constraints");
                return;
            }
        }
    }
    else
    {
        // Checks that we have ended inside the goal constraints
        bool inside_goal_constraints = true;
        for (size_t i = 0; i < msg->feedback.joint_names.size() && inside_goal_constraints; ++i)
        {
            double abs_error = fabs(msg->feedback.error.positions[i]);
            double goal_constraint = goal_constraints_[msg->feedback.joint_names[i]];
            if (goal_constraint >= 0 && abs_error > goal_constraint)
                inside_goal_constraints = false;

            // It's important to be stopped if that's desired.
            if (fabs(msg->feedback.desired.velocities[i]) < 1e-6)
            {
                if (fabs(msg->feedback.actual.velocities[i]) > stopped_velocity_tolerance_)
                    inside_goal_constraints = false;
            }
        }

        if (inside_goal_constraints)
        {
            active_goal_.setSucceeded();
            has_active_goal_ = false;
        }
        else if (now < end_time + ros::Duration(goal_time_constraint_))
        {
            // Still have some time left to make it.
        }
        else
        {
            ROS_WARN("Aborting because we wound up outside the goal constraints");
            active_goal_.setAborted();
            has_active_goal_ = false;
        }

    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "joint_trajecotry_controller");
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    kinova::JointTrajectoryActionController jtac(node);
    std::cout << "hello world!" << std::endl;

    ros::spin();
    return 0;
}
