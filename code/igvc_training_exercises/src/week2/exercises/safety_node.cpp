#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <iostream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "safety_node");
    ros::NodeHandle nh;

    ros::Publisher integer_pub = nh.advertise<std_msgs::Int32>("my_number", 1);
//    ros::Publisher even_publisher = nh.advertise<std_msgs::Int32>("even_number", 1);

    ros::Publisher g_even_publisher = nh.advertise<std_msgs::Int32>("even_number", 1);

    ros::Duration(1).sleep();

    std_msgs::Int32 message;
    message.data = 13;
    if ((message.data % 2) == 0) {
        g_even_publisher.publish(message);
    } else {
        integer_pub.publish(message);
    }

//    std_msgs::Int32 message;
//    message.data = 13;
//    integer_pub.publish(message);

    std::cout << "Hello World!" << std::endl;
    ros::spin();
}