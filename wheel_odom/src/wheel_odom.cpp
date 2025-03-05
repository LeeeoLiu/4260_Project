#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h> 

// parameter of robot
const double WHEEL_RADIUS = 0.033; 
const double WHEEL_BASE   = 0.287; 
double last_left_position = 0.0;
double last_right_position = 0.0;

// Global pose（x, y, theta）
double x = 0.0;
double y = 0.0;
double theta = 0.0;

// Global publisher
ros::Publisher path_pub;    
ros::Publisher odom_pub;   
nav_msgs::Path path;

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    // In gazebo simulation
    double right_position = msg->position[0];
    double left_position  = msg->position[1];

    // In real-world robot experiment
    // double left_position = msg->position[0];
    // double right_position  = msg->position[1];

    // TODO:
    // calculate the increment of two wheels
    double right_increment = right_position - last_right_position;
    double left_increment = left_position - last_left_position;

    // update position
    last_right_position = right_position;
    last_left_position = left_position;

    // calculate the displacement of two wheels

    // calculate the linear distance and rotation angle
    double linear_displacement = (right_increment + left_increment)/2;
    double angular_displacement = (right_increment - left_increment)/WHEEL_BASE;
    // update the global pose
    x = x + linear_displacement*cos(theta+angular_displacement/2);
    y = y + linear_displacement*sin(theta+angular_displacement/2);
    theta = theta + angular_displacement;
    // publish the path
    path_pub.publish(path);
    // update the path
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = x;
    this_pose_stamped.pose.position.y = y;
    this_pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
    path.oses.push_back(this_pose_stamped);
    // publish the wheel odom
    nav_msgs::Odometry wheel_odom;
    wheel_odom.pose.pose.position.x = x;
    wheel_odom.pose.pose.position.y = y;
    wheel_odom.pose.pose.orientation.z = theta;
    odom_pub.publish(wheel_odom);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wheel_odometry_node");
    ros::NodeHandle nh;

    // path_pub and odom_pub
    path_pub = nh.advertise<nav_msgs::Path>("robot_path", 10);
    odom_pub = nh.advertise<nav_msgs::Odometry>("wheel_odom", 10);

    // subscribe the joint states
    ros::Subscriber joint_state_sub = nh.subscribe("/joint_states", 10, jointStateCallback);

    ros::spin();
    return 0;
}

