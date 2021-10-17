import rospy
from laser_geometry import LaserProjection
from sensor_msgs.msg import PointCloud2, LaserScan

rospy.init_node("laserscan_to_pointcloud")
lp = LaserProjection()
pc_pub = rospy.Publisher("converted_pc", PointCloud2, queue_size=1)

def run(msg):
    pc2_msg = lp.projectLaser(msg)
    pc_pub.publish(pc2_msg)

rospy.Subscriber("/iris_laser/laser/scan", LaserScan, run, queue_size=1)
rospy.spin()