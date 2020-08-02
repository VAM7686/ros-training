#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>

void stringCallback(std_msgs::String message) {
    ROS_WARN_STREAM("I received a message: " << message.data);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "subscriber");
    ros::NodeHandle nh;

    ros::Subscriber string_sub = nh.subscribe("warning", 1, stringCallback);

    ros::spin();

}

