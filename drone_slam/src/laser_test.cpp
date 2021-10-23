#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>


void cb(const sensor_msgs::LaserScan::ConstPtr &scan) {
    std::cout << scan->ranges.size() << std::endl;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "read_laser_node");
    ros::NodeHandle n;
    
    ros::Subscriber sub = n.subscribe("/laser/scan", 1, cb);
    ros::spin();
}