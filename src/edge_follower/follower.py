from __future__ import division
import rospy
import numpy as np
import tf
import csv

import ur_robot as ur

saved_cloudXYZ = None
cloud_csv = './src/edge_follower/save_cloud.csv'
write_once = False

# Save the captured pointcloud
def freeze_cloud(freeze,cloudXYZ):
    global saved_cloudXYZ
    if freeze and saved_cloudXYZ is None:
        global saved_cloudXYZ
        saved_cloudXYZ = cloudXYZ
    return saved_cloudXYZ

# A constant pointcloud for ur robot to follow   
# Control the UR robot to 'scan' from left to right
def scan_mode(solidcloud,x_min,x_max,y_min,y_max,z_min,z_max):

    if solidcloud is not None:
        # Go through the pointcloud
        print("Size of cloud:", solidcloud.shape)
        for i in range(0,solidcloud.shape[0]):
            point = solidcloud[i,:]
            x,y,z = point[0],point[1],point[2]
            print("[{}]: x:{} y:{} z:{}".format(i,x,y,z))
            # Remove extreme points
            if((x>=x_min)and(x<=x_max))and((y>=y_min)and(y<=y_max))and((z>=z_min)and(z<=z_max)):
                ur.ur3_set_end_effector_goal_quat(x,y,z,ur.init_pose[3],\
                        ur.init_pose[4],ur.init_pose[5],ur.init_pose[6])


# A constant pointcloud for ur robot to follow   
# Control the robot to follow the path
def follow_mode(solidcloud,x_min,x_max,y_min,y_max,z_min,z_max):
    global write_once
    if solidcloud is not None:
        # Go through the pointcloud
        print("Size of cloud:", solidcloud.shape)
        for i in range(0,solidcloud.shape[0]):
            point = solidcloud[i,:]
            x,y,z = point[0],point[1],point[2]
            # Save to csv
            if not write_once:
                save_cloud(x,y,z)
            # Remove extreme points
            if((x>=x_min)and(x<=x_max))and((y>=y_min)and(y<=y_max))and((z>=z_min)and(z<=z_max)):
                print("[{}]: x:{} y:{} z:{}".format(i,x,y,z))
                ur.ur3_set_end_effector_goal_quat(x,y,z,ur.init_pose[3],\
                        ur.init_pose[4],ur.init_pose[5],ur.init_pose[6]) 
        write_once = True     

# Save the pointcloud into csv
def save_cloud(x,y,z):
    # the a is for append, if w for write is used then it overwrites the file
    f = open(cloud_csv, mode='w')
    with f as cloud_file:
        cloud_write = csv.writer(cloud_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        cloud_write.writerow([x,y,z])

    
        


