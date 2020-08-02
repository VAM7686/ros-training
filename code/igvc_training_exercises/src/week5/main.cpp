#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>

ros::Publisher g_map_pub;
nav_msgs::OccupancyGrid g_map;

void update_map(const pcl::PointCloud<pcl::PointXYZ>& pointcloud) {
    static tf::TransformListener g_transform_listener;

    // Convert timestamp from PCL (uint64) to ros::Time
    ros::Time timestamp = pcl_conversions::fromPCL(pointcloud.header.stamp);

    if (!g_transform_listener.waitForTransform("odom", "oswin", timestamp, ros::Duration(5)))
    {
        ROS_ERROR_STREAM("Transform error from odom to oswin");
        return;
    }
    tf::StampedTransform transform_to_odom;
    g_transform_listener.lookupTransform("odom", "oswin", timestamp, transform_to_odom);

    // Transform the pointcloud from the lidar frame to the odom frame using the transform form just now
    pcl::PointCloud<pcl::PointXYZ> transformed_pointcloud;
    pcl_ros::transformPointCloud(pointcloud, transformed_pointcloud, transform_to_odom);

    // Find the coordinate in the grid from each point in the pointcloud
    for (const auto& point : transformed_pointcloud)
    {
        int map_x = static_cast<int>(std::round((point.x - g_map.info.origin.position.x) / g_map.info.resolution));
        int map_y = static_cast<int>(std::round((point.y - g_map.info.origin.position.y) / g_map.info.resolution));

        // Convert the indices from 2D index to 1D index
        int index = map_y * g_map.info.width + map_x;

        // If it's less than 100, increment it
        if (g_map.data[index] < 100)
        {
            g_map.data[index]++;
        }
    }
}

void pointcloudCallback(const pcl::PointCloud<pcl::PointXYZ>& pointcloud)
{
    g_map.header.stamp = pcl_conversions::fromPCL(pointcloud.header.stamp);
    update_map(pointcloud);
    // Publish the updated map
    g_map_pub.publish(g_map);
    ROS_INFO_STREAM("Publishing Map");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "week5");

    ros::NodeHandle nh;

    //Subscribe to /oswin/pointcloud
    ros::Subscriber pointcloud_sub = nh.subscribe("/oswin/pointcloud", 1, pointcloudCallback);

    //Publisher of type nav_msgs::OccupancyGrid that publishes map
    g_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/oswin/map", 1);

    // Set information about the map, ie. width, height etc
    g_map.info.width = 200;
    g_map.info.height = 200;
    g_map.info.origin.position.x = -10;
    g_map.info.origin.position.y = -10;
    g_map.info.origin.orientation = tf::createQuaternionMsgFromYaw(0.0);
    g_map.info.resolution = 0.1;
    g_map.header.frame_id = "odom"; // Make sure to set the frame_id

    int total_cells = g_map.info.width * g_map.info.height;

    // Instantiate the g_map.data to have total_cells number of cells
    g_map.data = std::vector<int8_t>(total_cells, 0);

    ros::spin();
}