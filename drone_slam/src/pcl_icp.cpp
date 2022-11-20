#include "drone_slam/pcl_icp.h"

dslam::Dslam::Dslam():nh(ros::NodeHandle())
{
    sub_cloud = nh.subscribe("/camera/depth/points", 1, &dslam::Dslam::cb_cloud, this);
    sub_odom = nh.subscribe("/mavros/local_position/odom", 1, &dslam::Dslam::cb_odom, this);
    pub_cloud = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("icp/cloud/output", 1);

    dslam::Dslam::initialize();
}


void dslam::Dslam::initialize()
{
    _cloud_in.reset(new pcl::PointCloud<pcl::PointXYZ>());
    cloud_in.reset(new pcl::PointCloud<pcl::PointXYZ>());
    cloud_out.reset(new pcl::PointCloud<pcl::PointXYZ>());

    icp.setMaxCorrespondenceDistance(0.05);
    icp.setMaximumIterations(25);
    icp.setTransformationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(1);

    // tree = new octomap::OcTree(0.05);

    robot_x = 0.0;
    robot_y = 0.0;
    robot_z = 0.0;
}


void dslam::Dslam::cb_odom(const nav_msgs::Odometry::ConstPtr& msg)
{
    data[0] = msg->pose.pose.orientation.x;
    data[1] = msg->pose.pose.orientation.y;
    data[2] = msg->pose.pose.orientation.z;
    data[3] = msg->pose.pose.orientation.w;
    
    Eigen::Quaterniond q(data[3], data[0], data[1], data[2]);
    Eigen::Isometry3d t(q);
    t(0,3) = msg->pose.pose.position.x;
    t(1,3) = msg->pose.pose.position.y;
    t(2,3) = msg->pose.pose.position.z;

    _t = t;
}


void dslam::Dslam::cb_cloud(const sensor_msgs::PointCloud2ConstPtr& next_input) 
{
    pcl::fromROSMsg(*next_input, *_cloud_in);

    ////////////////////////////////////////////////////////////////////
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*_cloud_in, *_cloud_in, indices);
    
    ////////////////////////////////////////////////////////////////////
    vg.setInputCloud(_cloud_in);
    vg.setLeafSize(0.2f, 0.2f, 0.2f);
    vg.filter(*cloud_in);

    ////////////////////////////////////////////////////////////////////
    pcl::PointCloud<pcl::PointXYZ>::Ptr _tmp(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*cloud_in, *_tmp, _t.matrix());
    *cloud_in = *_tmp;

    if (cloud_out->points.size() != 0) 
    {
        pcl::PointCloud<pcl::PointXYZ> tmp;
        icp.setInputSource(cloud_in);
        icp.setInputTarget(cloud_out);
        icp.align(tmp);
        *cloud_out = tmp;
    } 
    else 
    {
        *cloud_out = *cloud_in;
    }

    ////////////////////////////////////////////////////////////////////
    // octomap::Pointcloud cloud_octo;
    // for (auto &p:cloud_in->points)
    // {
    //     cloud_octo.push_back(p.x, p.y, p.z);
    // }
    // tree->insertPointCloud(cloud_octo, octomap::point3d(_t(0,3), _t(1,3), _t(2,3)));
    // tree->updateInnerOccupancy();

    // octomap_msgs::Octomap _output;
    // octomap_msgs::fullMapToMsg(*tree, _output);

    ////////////////////////////////////////////////////////////////////
    pcl::toROSMsg(*cloud_out, output);
    pub_cloud.publish(output);
}


int main(int argc, char** argv) 
{
    ros::init(argc, argv, "icp");
    std::cout<<"Ready"<<std::endl;
    dslam::Dslam s;
    ros::spin();
}
