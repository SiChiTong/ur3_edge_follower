#!/usr/bin/env python
from std_msgs.msg import String
from math import pi
from sensor_msgs.msg import PointCloud2
import rospy
import pcl
import sensor_msgs.point_cloud2 as pc2
import ros_numpy
import numpy as np
import follower
import ur_robot as ur

# ROS node and topics
node_name = 'edge_follower'
target_cloud_topic = "/organized_edge_detector/output_occluding_edge"

# Euler Quaternion Transformation
def euler_to_quaternion(Yaw, Pitch, Roll):
  yaw   = Yaw   * pi / 180 
  pitch = Roll  * pi / 180 
  roll  = Pitch * pi / 180 

  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

  return [qx, qy, qz, qw]

# Geting PointCloud2 Data from ROS
def cloudXYZ_callback(data):
    # Pointcloud in array form
    pointcloudXYZ_np = None
    # Convert PointCloud2 to numpy array
    pointcloudXYZ = ros_numpy.numpify(data)
    # Generate an zeros array that match the dimensions of pc
    pointcloudXYZ_np=np.zeros((pointcloudXYZ.shape[0],3))
    # Fill in the data
    pointcloudXYZ_np[:,0]=pointcloudXYZ['x']
    pointcloudXYZ_np[:,1]=pointcloudXYZ['y']
    pointcloudXYZ_np[:,2]=pointcloudXYZ['z']

    # Feed into follower and it will guide the robot
    follower.guide(pointcloudXYZ_np,ur.capture_ready)

def main():
    # ROS Node Init
    rospy.init_node(node_name, anonymous=True)
    # PointCloud Subscriber
    rospy.Subscriber(target_cloud_topic, PointCloud2, cloudXYZ_callback)
    # Init UR robot and set to ready pose
    ur.ur3_init()
    ur.ur3_set_end_effector_goal_quat(ur.init_pose[0],ur.init_pose[1],ur.init_pose[2],ur.init_pose[3],\
            ur.init_pose[4],ur.init_pose[5],ur.init_pose[6])
    ur.capture_ready = True
    # Make sure the node is running
    rospy.spin()

if __name__ == '__main__':
    main()



