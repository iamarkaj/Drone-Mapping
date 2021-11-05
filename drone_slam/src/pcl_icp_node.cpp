#include <ros/ros.h>
#include <drone_slam/pcl_icp.h>

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "pcl_icp");
    ros::NodeHandle nh;
    std::cout<<"Starting..."<<std::endl;
    Slam s;
    ros::spin();
    return 0;
}