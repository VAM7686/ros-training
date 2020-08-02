#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>

ros::Publisher g_error_pub;

//Publisher for /oswin/velocity topic to control bottom turtle
ros::Publisher g_velocity_pub;

//Message from kyle ground truth
geometry_msgs::PoseStamped g_kyle_pose;

//Variable to store previous time
ros::Time g_prev_time;

//variable to store coefficients
double g_kp = 10;
double g_ki = 1;
double g_kd = 1;


void kylePoseCallback(geometry_msgs::PoseStamped msg)
{
    g_kyle_pose = msg;
}

void oswinPoseCallback(geometry_msgs::PoseStamped msg)
{
    bool g_initialized = (g_prev_time.sec == 0);

    if (g_initialized)
    {
        g_prev_time = ros::Time::now();
        return;
    }

    double error = g_kyle_pose.pose.position.x - msg.pose.position.x;

    ros::Duration delta_t = msg.header.stamp - g_prev_time;

    double accumulator;
    accumulator += error * delta_t.toSec();
    double integral = g_ki * accumulator;
    //double heading_error = angles::normalize_angle(std::atan2(dy, dx) - oswin_yaw));

    double proportional = error;
    double control = g_kp * proportional + integral;

    geometry_msgs::Twist twist_msg;
    twist_msg.linear.x = control;
    g_velocity_pub.publish(twist_msg);

    g_prev_time = msg.header.stamp;

    std_msgs::Float64 error_msg;
    error_msg.data = error;
    g_error_pub.publish(error_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "week3");

    ros::NodeHandle nh;

    //need to this to refer to the parameters that are placed between the <node> tags in a .launch file
    ros::NodeHandle pnh{"~"};
    pnh.getParam("kp", g_kp);
    ROS_INFO_STREAM("kp: " << g_kp);
    pnh.getParam("ki", g_ki);
    ROS_INFO_STREAM("ki: " << g_ki);

    g_velocity_pub = nh.advertise<geometry_msgs::Twist>("oswin/velocity", 1);
    g_error_pub = nh.advertise<std_msgs::Float64>("error", 1);

    //Subscribing to oswin and kyle ground_truth
    ros::Subscriber kyle_sub = nh.subscribe("kyle/ground_truth", 1, kylePoseCallback);
    ros::Subscriber oswin_sub = nh.subscribe("oswin/ground_truth", 1, oswinPoseCallback);
    ros::spin();
}
