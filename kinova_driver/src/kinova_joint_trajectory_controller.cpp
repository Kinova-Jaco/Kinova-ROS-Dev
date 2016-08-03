#include <kinova_driver/kinova_joint_trajectory_controller.h>
#include <std_msgs/Float64.h>
#include <angles/angles.h>
#include <ros/console.h>

using namespace kinova;

JointTrajectoryController::JointTrajectoryController(kinova::KinovaComm &kinova_comm, ros::NodeHandle& n):
    kinova_comm_(kinova_comm),
    nh_(n)
{
    ROS_DEBUG_STREAM_ONCE("Get in: " << __PRETTY_FUNCTION__);

    ros::NodeHandle pn("~");

    // Display debug information in teminal
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }

    sub_command_ = nh_.subscribe("/trajectory_controller/command", 1, &JointTrajectoryController::commandCB, this);

    pub_joint_feedback_ = nh_.advertise<control_msgs::FollowJointTrajectoryFeedback>("/trajectory_controller/state", 1);

    traj_frame_id_ = "root";
    number_joint_ = 6;
    joint_names_.resize(number_joint_);
    std::cout << "joint names in feedback of trajectory state are: " << std::endl;
    for (uint i = 0; i<joint_names_.size(); i++)
    {
        joint_names_[i] = "j2n6s300_joint_" + boost::lexical_cast<std::string>(i+1);
        std::cout << joint_names_[i] << " ";
    }
    std::cout << std::endl;

    timer_update_state_ = nh_.createTimer(ros::Duration(0.1), &JointTrajectoryController::update_state_timer, this);
    terminate_thread_ = false;

    thread_update_state_ = new boost::thread(boost::bind(&JointTrajectoryController::update_state, this));

    traj_feedback_msg_.joint_names.resize(joint_names_.size());
    traj_feedback_msg_.desired.positions.resize(joint_names_.size());
    traj_feedback_msg_.desired.velocities.resize(joint_names_.size());
    traj_feedback_msg_.actual.positions.resize(joint_names_.size());
    traj_feedback_msg_.actual.velocities.resize(joint_names_.size());
    traj_feedback_msg_.error.positions.resize(joint_names_.size());
    traj_feedback_msg_.error.velocities.resize(joint_names_.size());
    traj_feedback_msg_.joint_names = joint_names_;

    ROS_DEBUG_STREAM_ONCE("Get out: " << __PRETTY_FUNCTION__);
}

JointTrajectoryController::~JointTrajectoryController()
{
    ROS_DEBUG_STREAM_ONCE("Get in: " << __PRETTY_FUNCTION__);
    ROS_WARN("destruction entered!");
    {
        boost::mutex::scoped_lock terminate_lock(terminate_thread_mutex_);
        terminate_thread_ = true;
    }

    sub_command_.shutdown();
    pub_joint_feedback_.shutdown();

    timer_update_state_.stop();
    thread_update_state_->join();
    delete thread_update_state_;
    ROS_DEBUG_STREAM_ONCE("Get out: " << __PRETTY_FUNCTION__);
}



void JointTrajectoryController::commandCB(const trajectory_msgs::JointTrajectoryConstPtr &traj_msg)
{
    ROS_DEBUG_STREAM_ONCE("Get in: " << __PRETTY_FUNCTION__);

ROS_DEBUG_STREAM("traj_msg->points[3].velocities[0] IS: " << traj_msg->points[3].velocities[0]);

    bool command_abort = false;

    // if receive new command, clear all trajectory and stop api
    kinova_comm_.stopAPI();
    if(!kinova_comm_.isStopped())
    {
        ros::Duration(0.01).sleep();
    }
    kinova_comm_.eraseAllTrajectories();

    kinova_comm_.startAPI();
    if(kinova_comm_.isStopped())
    {
        ros::Duration(0.01).sleep();
    }

    traj_command_points_ = traj_msg->points;
    ROS_INFO_STREAM("Trajectory controller Receive trajectory with points number: " << traj_command_points_.size());

    // Map the index in joint_names and the msg
    std::vector<int> lookup(number_joint_, -1);

    for (size_t j = 0; j<number_joint_; j++)
    {
        for (size_t k = 0; k<traj_msg->joint_names.size(); k++)
            if(traj_msg->joint_names[k] == joint_names_[j]) // find joint_j in msg;
            {
                lookup[j] = k;
                break;
            }

        if (lookup[j] == -1) // if joint_j not found in msg;
        {
            std::string error_msg = "Joint name : " + joint_names_[j] + " not found in the msg.";
            ROS_ERROR("%s", error_msg.c_str());
            command_abort = true;
            return;
        }
    }

    // find out the duration of each segment in the traje
    std::vector<double> durations(number_joint_, 0.0); // computed by time_from_start
    double trajectory_duration = traj_command_points_[0].time_from_start.toSec();

    durations[0] = trajectory_duration;
    for (int i = 1; i<number_joint_; i++)
    {
        durations[i] = (traj_command_points_[i].time_from_start - traj_command_points_[i-1].time_from_start).toSec();
        trajectory_duration += durations[i];
    }

    // check msg validation
    for (size_t j = 0; j<traj_command_points_.size(); j++)
    {
        // position should not be empty
        if (traj_command_points_[j].positions.empty()) // find joint_j in msg;
        {
            ROS_ERROR_STREAM("Positions in trajectory command cannot be empty at point: " << j);
            command_abort = true;
            break;
        }
        // position size match
        if (traj_command_points_[j].positions.size() != number_joint_)
        {
            ROS_ERROR_STREAM("Positions at point " << j << " has size " << traj_command_points_[j].positions.size() << " in trajectory command, which does not match joint number! ");
            command_abort = true;
            break;
        }


        // if velocity provided, size match
        if (!traj_command_points_[j].velocities.empty() && traj_command_points_[j].positions.size() != number_joint_)
        {
            ROS_ERROR_STREAM("Positions at point " << j << " has size " << traj_command_points_[j].positions.size() << " in trajectory command, which does not match joint number! ");
            command_abort = true;
            break;
        }
    }

    if(command_abort)
        return;

    KinovaAngles traj_angle_command; // extract angle command from trajectory received.
    KinovaAngles kinova_angle_command; // store angle command sent to robot, with shortest distance
    kinova_angle_command.InitStruct(); // set velocity command to zeros;

    for (size_t i = 0; i<traj_command_points_.size(); i++)
    {
        traj_angle_command.Actuator1 = traj_command_points_[i].velocities[0] *180/M_PI;
        traj_angle_command.Actuator2 = traj_command_points_[i].velocities[1] *180/M_PI;
        traj_angle_command.Actuator3 = traj_command_points_[i].velocities[2] *180/M_PI;
        traj_angle_command.Actuator4 = traj_command_points_[i].velocities[3] *180/M_PI;
        traj_angle_command.Actuator5 = traj_command_points_[i].velocities[4] *180/M_PI;
        traj_angle_command.Actuator6 = traj_command_points_[i].velocities[5] *180/M_PI;

        // no need for: kinova_angle_command.applyShortestAngleDistanceTo(traj_angle_command);
        kinova_angle_command = traj_angle_command;

        ROS_INFO_STREAM("add kinova_angle_command " << i << " to trajectory list: ");
        kinova_comm_.printAngles(kinova_angle_command);

        // add joint_angles to Trajectory list
        //        kinova_comm_.setJointAngles(kinova_angle_command, 0, false);
        kinova_comm_.setJointVelocities(kinova_angle_command);
    }


    //    while(traj_command_points_.size()-- >= 0)
    //    {
    //        kinova_comm_.setJointVelocities(kinova_angle_command, 0, false);
    //        ros::Duration(time_from_start_ - previous_time_from_start_).sleep();
    //    }

    ROS_DEBUG_STREAM_ONCE("Get out: " << __PRETTY_FUNCTION__);
}


void JointTrajectoryController::update_state_timer(const ros::TimerEvent&)
{
}

void JointTrajectoryController::update_state()
{
    ROS_DEBUG_STREAM_ONCE("Get in: " << __PRETTY_FUNCTION__);

    ros::Rate update_rate(10);
    previous_pub_ = ros::Time::now();
    while (nh_.ok())
    {
        // check if terminate command is sent from main thread
        {
            boost::mutex::scoped_lock terminate_lock(terminate_thread_mutex_);
            if (terminate_thread_)
            {
                break;
            }
        }

        traj_feedback_msg_.header.frame_id = traj_frame_id_;
        traj_feedback_msg_.header.stamp = ros::Time::now();
        KinovaAngles current_joint_angles;
        KinovaAngles current_joint_velocity;
        AngularPosition current_joint_command;

        kinova_comm_.getAngularCommand(current_joint_command);
        kinova_comm_.getJointAngles(current_joint_angles);
        kinova_comm_.getJointVelocities(current_joint_velocity);


        traj_feedback_msg_.desired.velocities[0] = current_joint_command.Actuators.Actuator1 *M_PI/180;
        traj_feedback_msg_.desired.velocities[1] = current_joint_command.Actuators.Actuator2 *M_PI/180;
        traj_feedback_msg_.desired.velocities[2] = current_joint_command.Actuators.Actuator3 *M_PI/180;
        traj_feedback_msg_.desired.velocities[3] = current_joint_command.Actuators.Actuator4 *M_PI/180;
        traj_feedback_msg_.desired.velocities[4] = current_joint_command.Actuators.Actuator5 *M_PI/180;
        traj_feedback_msg_.desired.velocities[5] = current_joint_command.Actuators.Actuator6 *M_PI/180;

        traj_feedback_msg_.actual.positions[0] = current_joint_angles.Actuator1 *M_PI/180;
        traj_feedback_msg_.actual.positions[1] = current_joint_angles.Actuator2 *M_PI/180;
        traj_feedback_msg_.actual.positions[2] = current_joint_angles.Actuator3 *M_PI/180;
        traj_feedback_msg_.actual.positions[3] = current_joint_angles.Actuator4 *M_PI/180;
        traj_feedback_msg_.actual.positions[4] = current_joint_angles.Actuator5 *M_PI/180;
        traj_feedback_msg_.actual.positions[5] = current_joint_angles.Actuator6 *M_PI/180;

        traj_feedback_msg_.actual.velocities[0] = current_joint_velocity.Actuator1 *M_PI/180;
        traj_feedback_msg_.actual.velocities[1] = current_joint_velocity.Actuator2 *M_PI/180;
        traj_feedback_msg_.actual.velocities[2] = current_joint_velocity.Actuator3 *M_PI/180;
        traj_feedback_msg_.actual.velocities[3] = current_joint_velocity.Actuator4 *M_PI/180;
        traj_feedback_msg_.actual.velocities[4] = current_joint_velocity.Actuator5 *M_PI/180;
        traj_feedback_msg_.actual.velocities[5] = current_joint_velocity.Actuator6 *M_PI/180;

        for (size_t j = 0; j<joint_names_.size(); j++)
        {
            traj_feedback_msg_.error.velocities[j] = traj_feedback_msg_.actual.velocities[j] - traj_feedback_msg_.desired.velocities[j];
        }

        //        ROS_WARN_STREAM("I'm publishing after second: " << (ros::Time::now() - previous_pub_).toSec());
        pub_joint_feedback_.publish(traj_feedback_msg_);
        previous_pub_ = ros::Time::now();
        update_rate.sleep();
    }
    ROS_DEBUG_STREAM_ONCE("Get out: " << __PRETTY_FUNCTION__);
}
