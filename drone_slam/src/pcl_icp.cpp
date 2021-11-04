#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>

// Demo PCL ICP ROS

ros::Publisher _pub;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>);


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& next_input) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>);;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>);;

    pcl::fromROSMsg (*next_input, *_cloud_in);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*_cloud_in, *_cloud_in, indices);

    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud(_cloud_in);
    vg.setLeafSize(0.2f, 0.2f, 0.2f);
    vg.filter(*cloud_in);

    if (cloud_out->points.size() != 0) {
        pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
        icp.setInputSource(cloud_in);
        icp.setInputTarget(cloud_out);

        icp.setMaxCorrespondenceDistance(0.05);
        icp.setMaximumIterations(25);
        icp.setTransformationEpsilon(1e-8);
        icp.setEuclideanFitnessEpsilon(1);

        pcl::PointCloud<pcl::PointXYZRGB> tmp;
        icp.align(tmp);
        *cloud_out = *cloud_out + tmp;

        std::cerr << cloud_out->points.size() << std::endl;

    } else {
        *cloud_out = *cloud_in;
    }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_out, output);
    _pub.publish(output);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "pcl_icp");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/camera/depth/points", 10, cloud_cb);
    _pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_icp_output", 1);

    std::cout << "Publishing\n";
    while(ros::ok()) ros::spin();

    ros::shutdown();
    std::cout << "\nClosed\n";
}