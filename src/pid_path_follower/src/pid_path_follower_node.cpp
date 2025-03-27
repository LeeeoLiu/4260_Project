#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <vector>
#include <std_msgs/Bool.h>

// PID parameter
double kp_heading_, ki_heading_, kd_heading_;
double kp_dist_, ki_dist_, kd_dist_;
double max_linear_speed_, max_angular_speed_;
double goal_tolerance_;
double integrated_heading_error_ = 0.0;
double last_heading_error_ = 0.0;
double integrated_dist_error_ = 0.0;
double last_dist_error_ = 0.0;

bool path_received_ = false;
size_t current_index_ = 0;
nav_msgs::Path global_path_;
ros::Publisher cmd_pub_;
ros::Publisher goal_pub;

// function
double getYawFromQuat(const geometry_msgs::Quaternion &q);
double normalizeAngle(double angle);
double pidComputeDist(double error);
double pidComputeHeading(double error);
void stopRobot();
void pathCallback(const nav_msgs::Path::ConstPtr &path_msg);
void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);

// receive path
void pathCallback(const nav_msgs::Path::ConstPtr &path_msg)
{
    if (path_msg->poses.empty())
    {
        ROS_WARN("Received an empty path.");
        return;
    }
    global_path_ = *path_msg;
    path_received_ = true;
    current_index_ = 0;
    ROS_INFO("Received global path, length = %zu", global_path_.poses.size());
}

// control part
/*
TODO:
get the error
pub cmd
*/
void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
    if (!path_received_ || global_path_.poses.empty())
        return;

    // 1. get the position (x, y, yaw)
    double rx = odom_msg->pose.pose.position.x;
    double ry = odom_msg->pose.pose.position.y;
    double ryaw = getYawFromQuat(odom_msg->pose.pose.orientation);

    // 2. if the robot arrive the goal, then stop
    geometry_msgs::PoseStamped &goal_pose = global_path_.poses.back();
    double gx = goal_pose.pose.position.x;
    double gy = goal_pose.pose.position.y;
    double dist_to_goal = std::hypot(gx - rx, gy - ry);
    if (dist_to_goal < goal_tolerance_)
    {

        stopRobot();
        ROS_INFO_THROTTLE(1.0, "Goal Reached, stopping robot.");
        std_msgs::Bool goal_reach;
        goal_reach.data = true;
        goal_pub.publish(goal_reach);
        return;
    }
    std_msgs::Bool goal_reach;
    goal_reach.data = false;
    goal_pub.publish(goal_reach);

    // 3. determine the goal
    if (current_index_ >= global_path_.poses.size())
    {
        stopRobot();
        return;
    }
    double tx = global_path_.poses[current_index_].pose.position.x;
    double ty = global_path_.poses[current_index_].pose.position.y;
    double dist_to_waypoint = std::hypot(tx - rx, ty - ry);
    if (dist_to_waypoint < 0.1 && current_index_ < global_path_.poses.size() - 1)
    {
        current_index_++;
    }

    // 4. get the error
    // TODO:
    double desired_yaw = std::atan2(ty - ry, tx - rx);
    double heading_error = normalizeAngle(desired_yaw - ryaw);
    double dist_error = dist_to_waypoint;

    // 5. PID control
    double linear_cmd = pidComputeDist(dist_error);
    double angular_cmd = pidComputeHeading(heading_error);

    // 6. speed limitation
    if (linear_cmd > max_linear_speed_)
        linear_cmd = max_linear_speed_;
    if (linear_cmd < -max_linear_speed_)
        linear_cmd = -max_linear_speed_;
    if (angular_cmd > max_angular_speed_)
        angular_cmd = max_angular_speed_;
    if (angular_cmd < -max_angular_speed_)
        angular_cmd = -max_angular_speed_;

    // 7. pub cmd
    // TODO:
    geometry_msgs::Twist cmd_msg;
    cmd_msg.linear.x = linear_cmd;
    cmd_msg.angular.z = angular_cmd;
    cmd_pub_.publish(cmd_msg);
}


// note that currently using position PID control, 
// which means it aims to reduce error to 0

// TODO: PID distance control
double pidComputeDist(double error)
{
    // P
    double p_term = kp_dist_ * error;

    // I
    integrated_dist_error_ += error;
    double i_term = ki_dist_ * integrated_dist_error_;

    // D
    double d_term = kd_dist_ * (error - last_dist_error_);
    last_dist_error_ = error;

    return p_term + i_term + d_term;
}

// TODO: PID Heading control
double pidComputeHeading(double error)
{
    // P
    double p_term = kp_heading_ * error;

    // I
    integrated_heading_error_ += error;
    double i_term = ki_heading_ * integrated_heading_error_;

    // D
    double d_term = kd_heading_ * (error - last_heading_error_);
    last_heading_error_ = error;

    return p_term + i_term + d_term;
}

// stop robot
void stopRobot()
{
    geometry_msgs::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.angular.z = 0.0;
    cmd_pub_.publish(stop_cmd);
}

// get yaw
double getYawFromQuat(const geometry_msgs::Quaternion &q)
{
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
    return yaw;
}

// norm
double normalizeAngle(double angle)
{
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pid_path_follower_node");

    ros::NodeHandle nh, private_nh;

    // load PID param
    /*
    TODO: you have to uncomment the params and replace the PID param * by your own

    // private_nh.param("kp_heading", kp_heading_, *);
    // private_nh.param("ki_heading", ki_heading_, *);
    // private_nh.param("kd_heading", kd_heading_, *);
    // private_nh.param("kp_dist", kp_dist_, *);
    // private_nh.param("ki_dist", ki_dist_, *);
    // private_nh.param("kd_dist", kd_dist_, *);

    */

    // tuning PID param
    private_nh.param("kp_heading", kp_heading_, 0.1);
    private_nh.param("ki_heading", ki_heading_, 0.1);
    private_nh.param("kd_heading", kd_heading_, 0.1);
    private_nh.param("kp_dist", kp_dist_, 0.1);
    private_nh.param("ki_dist", ki_dist_, 0.1);
    private_nh.param("kd_dist", kd_dist_, 0.1);

    private_nh.param("max_linear_speed", max_linear_speed_, 0.1);
    private_nh.param("max_angular_speed", max_angular_speed_, 0.22);
    private_nh.param("goal_tolerance", goal_tolerance_, 0.2);

    ros::Subscriber path_sub = nh.subscribe("global_path", 1, pathCallback);
    ros::Subscriber odom_sub = nh.subscribe("odom", 1, odomCallback);
    cmd_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    goal_pub = nh.advertise<std_msgs::Bool>("goal_reached", 1);

    ROS_INFO("PID Path Follower Initialized.");
    ros::spin();
    return 0;
}
