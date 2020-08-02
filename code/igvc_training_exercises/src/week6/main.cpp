#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>

tf::TransformListener g_transform_listener;

tf::StampedTransform g_oswin_transform_to_odom;

tf::StampedTransform g_kyle_transform_to_odom;

//Publisher for /oswin/velocity topic
ros::Publisher g_velocity_pub;

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
State g_state_path;

int costCalculator(tf::StampedTransform oswin, tf::StampedTransform kyle, nav_msgs::OccupancyGrid grid, int j) {
    //assigning angular velocity from imu z axis angular velocity sensor
    g_state_path.twist.angular = j;

    //Keeping track of old velocity
    g_state_path.twist.linear_old = g_state_path.twist.linear;

    //Integrating acceleration to find velocity
    g_state_path.twist.linear = 1;

    //Integrating angular velocity to find heading
    g_state_path.pose.heading += g_state_path.twist.angular * 1;

    //Finding average velocity
    g_state_path.twist.linear_avg = (g_state_path.twist.linear + g_state_path.twist.linear_old) / 2;

    //Finding x and y position
    g_state_path.pose.position.x += cos(g_state_path.pose.heading) * g_state_path.twist.linear_avg * 1;
    g_state_path.pose.position.y += sin(g_state_path.pose.heading) * g_state_path.twist.linear_avg * 1;

    int map_x = static_cast<int>(std::round((g_state_path.pose.position.x - grid.info.origin.position.x) / grid.info.resolution));
    int map_y = static_cast<int>(std::round((g_state_path.pose.position.y - grid.info.origin.position.y) / grid.info.resolution));

    int kyle_dist_x = kyle.getOrigin().x() - oswin.getOrigin().x();
    int kyle_dist_y = kyle.getOrigin().y() - oswin.getOrigin().y();

    // Convert the indices from 2D index to 1D index
    int index = map_y * grid.info.width + map_x;

    if (grid.data[index] >= 75) {
        return INT_MAX;
    } else {
        return sqrt(pow(kyle_dist_x, 2) + pow(kyle_dist_y, 2));
    }

}

void mapCallback(const nav_msgs::OccupancyGrid grid)
{
    // get time
    ros::Time timestamp = grid.info.map_load_time;

    //get locations of oswin and kyle in relation to odom
    g_transform_listener.lookupTransform("odom", "oswin", timestamp, g_oswin_transform_to_odom);
    g_transform_listener.lookupTransform("odom", "kyle", timestamp, g_kyle_transform_to_odom);

    //number of steps variable
    int num_steps = 20;

    //loop
    static int costs[9] = { };
    for (int i = -4; i <= 4; i++) {
        for (int j = 0; j <= num_steps; j++) {
            costs[i + 4] += costCalculator(g_oswin_transform_to_odom, g_kyle_transform_to_odom, grid, j);
        }
    }
    int leastCost = 0;
    int linear_velocity = 0;
    for(int i = 0; i < 9; i++) {
        if (leastCost < costs[i]) {
            leastCost = costs[i];
            linear_velocity = (i - 4);
        }
    }

    geometry_msgs::Twist twist_msg;
    twist_msg.angular.z = linear_velocity;
    g_velocity_pub.publish(twist_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "week6");

    ros::NodeHandle nh;

    //Subscribe to /oswin/map that week 5 node is publishing
    ros::Subscriber map_sub = nh.subscribe("/oswin/map", 1, mapCallback);

    ros::spin();
}