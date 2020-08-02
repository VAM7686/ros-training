#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <iostream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "week2");
    ros::NodeHandle nh;

    ros::Publisher integer_pub = nh.advertise<std_msgs::Int32>("my_number", 100);

    ros::Duration(1).sleep();

    for (int i = 0; i < 100; i++) {
        std_msgs::Int32 message;
        message.data = i;
        integer_pub.publish(message);
    }
//    std_msgs::Int32 message;
//    message.data = 13;
//    integer_pub.publish(message);

    std::cout << "Hello World!" << std::endl;
    ros::spin();
}