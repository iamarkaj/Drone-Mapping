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
    
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud_in, cloud_in, cloud_out;

    laser_geometry::LaserProjection projector;
    sensor_msgs::PointCloud2 next_input, output;
    
    float robot_x, robot_y, robot_z;

    void initialize();
    void cb_odom(const nav_msgs::Odometry::ConstPtr& msg);
    void cb_laser(const sensor_msgs::LaserScan::ConstPtr& scan_in);
};

#endif