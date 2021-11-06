#include "drone_slam/pcl_icp.h"


Slam::Slam():_nh(ros::NodeHandle())
{
    _sub_laser = _nh.subscribe("/laser/scan", 100, &Slam::cb_laser, this);
    _sub_odom = _nh.subscribe("/mavros/local_position/odom", 100, &Slam::cb_odom, this);
    _pub = _nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("pcl_icp_output", 10);

    Slam::initialize();
}

void Slam::initialize()
{
    _cloud_in.reset(new pcl::PointCloud<pcl::PointXYZ>());
    cloud_in.reset(new pcl::PointCloud<pcl::PointXYZ>());
    cloud_out.reset(new pcl::PointCloud<pcl::PointXYZ>());

    vg.setInputCloud(_cloud_in);
    vg.setLeafSize(0.01f, 0.01f, 0.01f);

    icp.setMaxCorrespondenceDistance(0.05);
    icp.setMaximumIterations(25);
    icp.setTransformationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(1);

    robot_x = 0.0;
    robot_y = 0.0;
    robot_z = 0.0;

}

void Slam::cb_odom(const nav_msgs::Odometry::ConstPtr& msg)
{
    robot_x = msg->pose.pose.position.x;
    robot_y = msg->pose.pose.position.y;
    robot_z = msg->pose.pose.position.z;
}


void Slam::cb_laser(const sensor_msgs::LaserScan::ConstPtr& scan_in) 
{
    projector.projectLaser(*scan_in, next_input);
    next_input.header.frame_id = "/laser";
    next_input.header.stamp = scan_in->header.stamp;

    pcl::fromROSMsg(next_input, *_cloud_in);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*_cloud_in, *_cloud_in, indices);
    
    vg.filter(*cloud_in);

    for(auto& p:cloud_in->points)
    {
        p.x += robot_x;
        p.y += robot_y;
        p.z += robot_z;
    }

    if (cloud_out->points.size() != 0) 
    {
        pcl::PointCloud<pcl::PointXYZ> tmp;
        icp.setInputSource(cloud_in);
        icp.setInputTarget(cloud_out);
        icp.align(tmp);
        *cloud_out = *cloud_out + tmp;

        std::cerr << cloud_out->points.size() << std::endl;
    } 
    else 
    {
        *cloud_out = *cloud_in;
    }

    pcl::toROSMsg(*cloud_out, output);
    _pub.publish(output);
}
