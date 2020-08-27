# ROS_COSTMAP_GENERATOR

Implemented a 2D Costmap Generator from scratch using C++ and ROS. This costmap takes in a predefined map, pointcloud output from either a 3D LiDAR or depth camera, and the robot's inner/outer radius, and generates a costmap of it's surroundings.

![](/Images_Costmap/costmap_with_cloud.png)
Example of a Costmap being generated from a 3D Pointcloud. The Robot's position is marked with a green square.

![](/Images_Costmap/costmap.png)
The Costmap shown without the pointcloud.

To view a sample robot generating a costmap as it traverses, run the command 
`rosrun costmap demo.launch`

Alternatively the node can be run on a custom robot and map using 

`rosrun costmap costmap_generator`

However all of the necessary sensor topics will have to be configured, along with robot inner and outer radius.