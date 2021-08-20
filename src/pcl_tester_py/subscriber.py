#!/usr/bin/env python
# If you do not have ros-numpy, please install it
# sudo apt-get install ros-melodic-ros-numpy
import rospy
import pcl
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import ros_numpy
import numpy as np

# Geting PointCloud2 Data from ROS
def callback(data):
    pc = ros_numpy.numpify(data)
    points=np.zeros((pc.shape[0],3))
    points[:,0]=pc['x']
    points[:,1]=pc['y']
    points[:,2]=pc['z']
    print(points)

rospy.init_node('listener', anonymous=True)
rospy.Subscriber("/organized_edge_detector/output_occluding_edge", PointCloud2, callback)
rospy.spin()