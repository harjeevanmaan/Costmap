#include "ros/ros.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <string>
#include <mutex>
#include <thread>
#include <iostream>
#include <math.h>
#include <costmap/costmap.h>


int main(int argc, char ** argv){
    ros::init(argc, argv, "costmap_generator");
    ros::NodeHandle nh("~");

    std::string map_server_name, pointcloud_topic, costmap_topic;
    float lower_thres, upper_thres, inner_rad, outer_rad;

    nh.getParam("map_server_name", map_server_name);
    nh.getParam("pointcloud_topic", pointcloud_topic);
    nh.getParam("costmap_topic", costmap_topic);
    nh.getParam("lower_thres", lower_thres);
    nh.getParam("upper_thres", upper_thres);
    nh.getParam("inner_rad", inner_rad);
    nh.getParam("outer_rad", outer_rad);

    Costmap cmap(map_server_name, pointcloud_topic, costmap_topic, lower_thres, upper_thres, inner_rad, outer_rad);
    ros::Rate loop_rate = ros::Rate(2);

    while(ros::ok()){
        loop_rate.sleep();
    }
    return 0;
}