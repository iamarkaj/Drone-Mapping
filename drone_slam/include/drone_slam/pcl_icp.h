#ifndef DRONE_SLAM_H
#define DRONE_SLAM_H

#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <boost/format.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

#include <laser_geometry/laser_geometry.h>

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap/math/Pose6D.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>


namespace dslam
{
    class Dslam 
    {
        public:
            Dslam();

        private:
            ros::NodeHandle nh;
            ros::Publisher pub_cloud;
            ros::Subscriber sub_cloud, sub_odom;
            
            pcl::VoxelGrid<pcl::PointXYZ> vg;
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud_in, cloud_in, cloud_out;

            // octomap::OcTree *tree;
            sensor_msgs::PointCloud2 output;
            Eigen::Isometry3d _t;


            float robot_x, robot_y, robot_z;
            std::vector<float> data = {0.0, 0.0, 0.0, 0.0};

            void initialize();
            void cb_odom(const nav_msgs::Odometry::ConstPtr& msg);
            void cb_cloud(const sensor_msgs::PointCloud2ConstPtr& next_input);
    };
}

#endif