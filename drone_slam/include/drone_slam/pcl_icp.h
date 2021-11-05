#ifndef DRONE_SLAM_H
#define DRONE_SLAM_H

#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <laser_geometry/laser_geometry.h>


class Slam 
{
public:
    Slam();

private:
    ros::NodeHandle _nh;
    ros::Publisher _pub;
    ros::Subscriber _sub_laser, _sub_odom;

    pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud_in, cloud_out, cloud_in;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    sensor_msgs::PointCloud2 next_input, output;
    laser_geometry::LaserProjection projector;

    float robot_x = 0.0, robot_y = 0.0, robot_z = 0.0;

    void cb_laser(const sensor_msgs::LaserScan::ConstPtr& scan_in) ;
    void cb_odom(const nav_msgs::Odometry::ConstPtr& msg);
};

#endif