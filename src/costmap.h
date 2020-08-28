#ifndef COSTMAP_H
#define COSTMAP_H

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

class Costmap{
private:
    std::string pointcloud_topic;
    ros::NodeHandle n;
    nav_msgs::OccupancyGrid static_map;
    nav_msgs::OccupancyGrid costmap;
    ros::Subscriber pc_sub;
    ros::Publisher costmap_pub;
    std::mutex mtx;
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud = nullptr;
    tf2_ros::Buffer tfbuffer;
    float inner_rad = 0.140;
    float outer_rad = 0.153;
    float lower_thres = 0.1;
    float upper_thres = 1;
    float update_freq = 5;
    ros::Rate loop_rate = ros::Rate(update_freq);
    ros::AsyncSpinner spinner = ros::AsyncSpinner(4);

public:

    Costmap(const std::string map_name);
    void load_map(const std::string map_name);
    void filter_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &pc_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &pc_filtered);
    void transform_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, geometry_msgs::TransformStamped &tf_st);
    void obstacle_layer_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
    int to_map_index(float x, float y);        //get map index, using x and y distance in meters from map origin
    bool is_valid(pcl::PointCloud<pcl::PointXYZ>::iterator &it);
    float hypotenuse(float x, float y);
    void inflate_obstacle(float x, float y);
    void update_map();         //update costmap with pointcloud data
    void check_callbacks();
};

#endif