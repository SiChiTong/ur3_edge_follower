from __future__ import division
import rospy
import numpy as np
import tf

import ur_robot as ur

saved_cloudXYZ = None

# Save the captured pointcloud
def freeze_cloud(freeze,cloudXYZ):
    global saved_cloudXYZ
    if freeze and saved_cloudXYZ is None:
        global saved_cloudXYZ
        saved_cloudXYZ = cloudXYZ
    return saved_cloudXYZ

# A constant pointcloud for ur robot to follow   
def guide(solidcloud):
    if solidcloud is not None:
        point = solidcloud[500,:]
        x,y,z = point[0],point[1],point[2]
        print("x:{} y:{} z:{}".format(x,y,z))

        ur.ur3_set_end_effector_goal_quat(x,y,(z+0.01),ur.init_pose[3],\
                ur.init_pose[4],ur.init_pose[5],ur.init_pose[6])

        point = solidcloud[400,:]
        x,y,z = point[0],point[1],point[2]
        print("x:{} y:{} z:{}".format(x,y,z))

        ur.ur3_set_end_effector_goal_quat(x,y,(z+0.01),ur.init_pose[3],\
                ur.init_pose[4],ur.init_pose[5],ur.init_pose[6])

        point = solidcloud[300,:]
        x,y,z = point[0],point[1],point[2]
        print("x:{} y:{} z:{}".format(x,y,z))

        ur.ur3_set_end_effector_goal_quat(x,y,(z+0.01),ur.init_pose[3],\
                ur.init_pose[4],ur.init_pose[5],ur.init_pose[6])

