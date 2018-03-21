#include <chrono>
#include <thread>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <Eigen/Core>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

ros::Publisher _trajectory_pub;

Eigen::Vector3d _current_position, _desired_position;
double _current_yaw, _desired_yaw;
double _vx, _vy, _vz, _yaw_rate;

void odometryCallback(const nav_msgs::OdometryConstPtr msg)
{
    // get current state
    _current_position(0) = msg->pose.pose.position.x;
    _current_position(1) = msg->pose.pose.position.y;
    _current_position(2) = msg->pose.pose.position.z;

    tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z,
                     msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    _current_yaw = yaw;

    // transform the command
    double off_x = _vx * cos(_current_yaw) - _vy * sin(_current_yaw);
    double off_y = _vx * sin(_current_yaw) + _vy * cos(_current_yaw);
 
    // Add some offset on current position based on keyboard command
    double kl = 1.0, ka = 1.0;
    _desired_position(0) = _current_position(0) + kl * off_x;
    _desired_position(1) = _current_position(1) + kl * off_y;
    _desired_position(2) = _current_position(2) + kl * _vz;

    _desired_yaw = _current_yaw + ka * _yaw_rate;

    // convert to trajectory msgs
    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    trajectory_msg.header.stamp = ros::Time::now();
    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(_desired_position, _desired_yaw, &trajectory_msg);
    _trajectory_pub.publish(trajectory_msg);
}

void keyboardCallback(const geometry_msgs::TwistConstPtr msg)
{
    // get keyboard command
    _vx = msg->linear.x;
    _vy = msg->linear.y;
    _vz = msg->linear.z;
    _yaw_rate = msg->angular.z;

    ROS_INFO_STREAM("Yaw is:" << _current_yaw << " ,"
                              << "Position is:" << _current_position.transpose());
    ROS_INFO("Publishing command on : [%f, %f, %f, %f].", _vx, _vy, _vz, _yaw_rate);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hovering_example");
    ros::NodeHandle nh;

    _trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
        mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
    ROS_INFO("Started hovering example.");

    // Subscribe to ground truth odometry and keyboard command
    ros::Subscriber odom_sub = nh.subscribe("/firefly/ground_truth/odometry", 5, odometryCallback);
    ros::Subscriber key_sub = nh.subscribe("/keyboard/twist", 5, keyboardCallback);

    // wait and unpause gazebo
    std_srvs::Empty srv;
    bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    unsigned int i = 0;
    // Trying to unpause Gazebo for 10 seconds.
    while(i <= 10 && !unpaused)
    {
        ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        unpaused = ros::service::call("/gazebo/unpause_physics", srv);
        ++i;
    }
    if(!unpaused)
    {
        ROS_FATAL("Could not wake up Gazebo.");
        return -1;
    }
    else
    {
        ROS_INFO("Unpaused the Gazebo simulation.");
    }
    // Wait for 5 seconds to let the Gazebo GUI show up.
    ros::Duration(5.0).sleep();

    ros::spin();

    return 0;
}
