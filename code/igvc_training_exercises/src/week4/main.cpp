#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

struct Position
{
    double x;
    double y;
    double z;
};

struct Pose
{
    Position position;
    double heading;
};

//Velocity in x, y, z
struct Velocity
{
    double x;
    double y;
};

// linear and angular velocity
struct Twist
{
    double linear;
    double linear_old;
    double linear_avg;
    double angular;
};

// State of robot
struct State
{
    Pose pose;
    Twist twist;
};

//Variable to store robot current info
State g_state;

//Variable to store previous time
ros::Time g_prev_time;

//Publisher to publish results of dead reckoning
ros::Publisher g_odometry_pub;

void oswinImuCall(sensor_msgs::Imu message){
    //ROS_INFO_STREAM("Linear acceleration x axis: " << message.linear_acceleration.x);

    static tf::TransformBroadcaster g_broadcaster;

    //Finding time difference from previous message to current message
    ros::Duration dt = message.header.stamp - g_prev_time;

    //assigning angular velocity from imu z axis angular velocity sensor
    g_state.twist.angular = message.angular_velocity.z;

    //Keeping track of old velocity
    g_state.twist.linear_old = g_state.twist.linear;

    //Integrating acceleration to find velocity
    g_state.twist.linear += message.linear_acceleration.x * dt.toSec();

    //Integrating angular velocity to find heading
    g_state.pose.heading += g_state.twist.angular * dt.toSec();

    //Finding average velocity
    g_state.twist.linear_avg = (g_state.twist.linear + g_state.twist.linear_old) / 2;

    //Finding x and y position
    g_state.pose.position.x += cos(g_state.pose.heading) * g_state.twist.linear_avg * dt.toSec();
    g_state.pose.position.y += sin(g_state.pose.heading) * g_state.twist.linear_avg * dt.toSec();

    //Keeping track of old time
    g_prev_time = message.header.stamp;

    //Determining position of robot to world and publishing
    nav_msgs::Odometry odometryMessage;
    odometryMessage.header.frame_id = "odom";
    odometryMessage.header.stamp = message.header.stamp;
    odometryMessage.pose.pose.position.x = g_state.pose.position.x;
    odometryMessage.pose.pose.position.y = g_state.pose.position.y;

    odometryMessage.pose.pose.orientation = tf::createQuaternionMsgFromYaw(g_state.pose.heading);
    odometryMessage.twist.twist.linear.x = g_state.twist.linear;
    odometryMessage.twist.twist.angular.z = g_state.twist.angular;

    g_odometry_pub.publish(odometryMessage);

    // Fill in the information for the tf::Transform variable
    tf::Transform transform;
    transform.setOrigin({g_state.pose.position.x, g_state.pose.position.y, 0.0});
    // Use tf::createQuaternionFromYaw to convert from a heading to a tf quaternion
    transform.setRotation(tf::createQuaternionFromYaw(g_state.pose.heading));
    tf::StampedTransform stamped_transform = tf::StampedTransform(transform, message.header.stamp, "odom", "oswin");
    g_broadcaster.sendTransform(stamped_transform);
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "week4");
    ros::NodeHandle nh;

    ros::Subscriber oswinImuSub = nh.subscribe("/oswin/imu", 1, oswinImuCall);

    g_odometry_pub = nh.advertise<nav_msgs::Odometry>("/oswin/odometry", 1);

    g_prev_time = ros::Time::now();

    ros::spin();
}