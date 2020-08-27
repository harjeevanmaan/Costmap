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

Costmap::Costmap(const std::string map_name, const std::string pc_topic, const std::string cmap_topic, 
                const float l_thres, const float u_thres, const float i_rad, const float o_rad) 
            : map_server_name(map_name), pointcloud_topic(pc_topic), costmap_topic(cmap_topic), 
            lower_thres(l_thres), upper_thres(u_thres), inner_rad(i_rad), outer_rad(o_rad){
    load_map(map_name);
    pc_sub = n.subscribe("/camera/depth/points", 1, &Costmap::obstacle_layer_callback, this);
    costmap_pub = n.advertise<nav_msgs::OccupancyGrid>("jeevon_costmap", 1);
    input_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    std::thread t(&Costmap::check_callbacks, this);
    t.detach();
}

void Costmap::load_map(const std::string map_name){
    ros::ServiceClient client = n.serviceClient<nav_msgs::GetMap>(map_name);
    nav_msgs::GetMap srv;
    client.call(srv);
            std::cout << srv.response.map.info.width << std::endl;
    while(srv.response.map.info.width == 0 && ros::ok()) {
        ros::Duration(0.2).sleep();
        client.call(srv);
        std::cout << srv.response.map.info.width << std::endl;

    }
    static_map = srv.response.map;
    costmap = static_map;
}

void Costmap::filter_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &pc_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &pc_filtered){
    pcl::VoxelGrid<pcl::PointXYZ> vox;
    vox.setInputCloud(pc_in);
    vox.setLeafSize(0.03, 0.03, 0.03);
    vox.filter(*pc_filtered);
}

void Costmap::transform_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, geometry_msgs::TransformStamped &tf_st){
    Eigen::Affine3d matrix = Eigen::Affine3d::Identity();
    matrix.translation() << tf_st.transform.translation.x, tf_st.transform.translation.y, tf_st.transform.translation.z;
    matrix.rotate(Eigen::Quaterniond(
        tf_st.transform.rotation.w,
        tf_st.transform.rotation.x,
        tf_st.transform.rotation.y,
        tf_st.transform.rotation.z
    ));
    pcl::transformPointCloud(*cloud, *input_cloud, matrix);
}

void Costmap::obstacle_layer_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg){
    std::lock_guard<std::mutex> lock(mtx);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_in (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    geometry_msgs::TransformStamped transformstamped;

    try{
        tf2_ros::TransformListener listener(tfbuffer);
        transformstamped = tfbuffer.lookupTransform("map", "camera_rgb_optical_frame", cloud_msg->header.stamp, ros::Duration(0.5));
    }

    catch(tf2::TransformException &ex){
        ROS_WARN("%s", ex.what());
        ros::Duration(1/update_freq).sleep();
        return;
    }

    pcl::fromROSMsg(*cloud_msg, *pc_in);            //convert to pcl type from ros type
    filter_cloud(pc_in, pc_filtered);               //voxel filter to increase processing time
    transform_cloud(pc_filtered, transformstamped);     //transform to map coordinate frame from sensor frame
    update_map();

}

int Costmap::to_map_index(float x, float y){         //get map index, using x and y distance in meters from map origin
    x -= static_map.info.origin.position.x;
    x /= static_map.info.resolution;
    y -= static_map.info.origin.position.y;
    y = round(y/static_map.info.resolution);

    return round(x + y*static_map.info.width); 
}

bool Costmap::is_valid(pcl::PointCloud<pcl::PointXYZ>::iterator &it){
    if(it->z < lower_thres || it->z > upper_thres) return false;           //launch file parameter
    return true;
}

float Costmap::hypotenuse(float x, float y){
    return sqrt(x*x + y*y);
}

void Costmap::inflate_obstacle(float x, float y){
    float resolution = costmap.info.resolution;
    auto curr_val = 0;
    for(float i=-1*outer_rad; i<outer_rad; i += resolution){
        for(float j=-1*outer_rad; j<outer_rad; j += resolution){
            int index = to_map_index(x+i, y+j);
            if(index < 0) continue;
            float distance = hypotenuse(i, j);
            int cost = -127 + distance/outer_rad*127;
            if(costmap.data[index] > cost) costmap.data[index] = cost;            //if current cost is greater (lower value)
        }
    }
}

void Costmap::update_map(){          //update costmap with pointcloud data
    if(input_cloud->width == 0) return;     //if empty cloud
    costmap = static_map;

    for(auto it = input_cloud->begin(); it!= input_cloud->end(); it++){
        if(!is_valid(it)) continue;
        costmap.data[to_map_index(it->x, it->y)] = -127;         //set as fatal obstacle
        inflate_obstacle(it->x, it->y);
    }
    costmap_pub.publish(costmap);
}

void Costmap::check_callbacks(){
    spinner.start();
    ros::waitForShutdown();
}