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
def guide(solidcloud,x_min,x_max,y_min,y_max):
    if solidcloud is not None:
        point = solidcloud[500,:]
        x,y,z = point[0],point[1],point[2]
        print("500: x:{} y:{} z:{}".format(x,y,z))
        if((x>=x_min)and(x<=x_max))and((y>=y_min)and(y<=y_max)):
            ur.ur3_set_end_effector_goal_quat(x,y,z,ur.init_pose[3],\
                    ur.init_pose[4],ur.init_pose[5],ur.init_pose[6])

        point = solidcloud[400,:]
        x,y,z = point[0],point[1],point[2]
        print("400: x:{} y:{} z:{}".format(x,y,z))
        if((x>=x_min)and(x<=x_max))and((y>=y_min)and(y<=y_max)):
            ur.ur3_set_end_effector_goal_quat(x,y,z,ur.init_pose[3],\
                    ur.init_pose[4],ur.init_pose[5],ur.init_pose[6])

        point = solidcloud[300,:]
        x,y,z = point[0],point[1],point[2]
        print("300: x:{} y:{} z:{}".format(x,y,z))
        if((x>=x_min)and(x<=x_max))and((y>=y_min)and(y<=y_max)):
            ur.ur3_set_end_effector_goal_quat(x,y,z,ur.init_pose[3],\
                    ur.init_pose[4],ur.init_pose[5],ur.init_pose[6])

