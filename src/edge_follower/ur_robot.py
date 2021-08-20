#!/usr/bin/env python
from std_msgs.msg import String
from math import pi
from sensor_msgs.msg import PointCloud2
import rospy
import sensor_msgs.point_cloud2 as pc2
import ros_numpy
import numpy as np
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

# UR Robot
robot = None
group = None
scene = None
display_trajectory_publisher = None
init_pose = (-0.2784,0.078,0.4,0.074,0.695,0.711,0.0717)
capture_ready = False

# UR Robot initialization
def ur3_init():
  global robot,group,scene,display_trajectory_publisher
  ## First initialize moveit_commander and rospy.
  #print "============ Starting system"
  moveit_commander.roscpp_initialize(sys.argv)
  #rospy.init_node('move_group_python_interface_tutorial',
  #                anonymous=True)

  ## Instantiate a RobotCommander object.  This object is an interface to
  ## the robot as a whole.
  robot = moveit_commander.RobotCommander()

  ## Instantiate a PlanningSceneInterface object.  This object is an interface
  ## to the world surrounding the robot.
  scene = moveit_commander.PlanningSceneInterface()

  ## Instantiate a MoveGroupCommander object.  This object is an interface
  ## to one group of joints.  In this case the group is the joints in the ur3
  ## arm.  This interface can be used to plan and execute motions on the ur3
  ## arm.
  group = moveit_commander.MoveGroupCommander("ur3")

  ## We create this DisplayTrajectory publisher which is used below to publish
  ## trajectories for RVIZ to visualize.
  display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory)

# Set robot goal based on end_effector
def ur3_set_end_effector_goal_quat(pos_x,pos_y,pos_z,qx,qy,qz,qw):
  global robot,group,scene,display_trajectory_publisher
  current_pose = group.get_current_pose().pose
  print ("Original Pose Information")
  print (current_pose)
  # XYZ are in terms of meters
  x = 0
  y = 0
  z = 0
  #Yaw,pitch and roll should be in degree    
  # They are all relative to base link coorindates     
  roll = 10
  yaw = 183
  pitch = 180

  pose_goal = geometry_msgs.msg.Pose()
  #Q = euler_to_quaternion(yaw , pitch, roll)
  #print Q
  pose_goal.orientation.x = qx
  pose_goal.orientation.y = qy
  pose_goal.orientation.z = qz
  pose_goal.orientation.w = qw

  pose_goal.position.x = pos_x
  pose_goal.position.y = pos_y
  pose_goal.position.z = pos_z
  group.set_pose_target(pose_goal)

  plan = group.go(wait=True)
  group.stop()
  current_pose = group.get_current_pose().pose
  print ("------------ Target Pose Information ------------")
  print (current_pose)
  group.clear_pose_targets()
  print ("[INFO] Goal of end effector is arrived")