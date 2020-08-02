#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <iostream>

void integerCallback(std_msgs::Int32 message) {
    ROS_INFO_STREAM("I received a message: " << message.data);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "subscriber");
    ros::NodeHandle nh;

    ros::Subscriber integer_sub = nh.subscribe("my_number", 1, integerCallback);

    ros::spin();

}

