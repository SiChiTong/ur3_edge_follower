from __future__ import division
import rospy
import numpy as np
import tf
import csv
from math import atan2

import ur_robot as ur

saved_cloudXYZ = None
cloud_csv = './src/edge_follower/save_cloud.csv'
write_once = False
intervals = 1

# Save the pointcloud into csv
def save_cloud(x,y,z):
    # the a is for append, if w for write is used then it overwrites the file
    f = open(cloud_csv, mode='w')
    with f as cloud_file:
        cloud_write = csv.writer(cloud_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        cloud_write.writerow([x,y,z])

# Save the captured pointcloud
def freeze_cloud(freeze,cloudXYZ):
    global saved_cloudXYZ
    if freeze and saved_cloudXYZ is None:
        global saved_cloudXYZ
        saved_cloudXYZ = cloudXYZ
    return saved_cloudXYZ

# Sorting operations
def argsort(seq):
    #http://stackoverflow.com/questions/3382352/equivalent-of-numpy-argsort-in-basic-python/3382369#3382369
    #by unutbu
    #https://stackoverflow.com/questions/3382352/equivalent-of-numpy-argsort-in-basic-python 
    # from Boris Gorelik
    return sorted(range(len(seq)), key=seq.__getitem__)

# Sort the points in the cloud according to clockwise or anti clockwise based on the angles
def rotational_sort(cloud,rot_center,clockwise=True):
    cx,cy=rot_center
    angles = [atan2(x-cx, y-cy) for x,y,z in cloud]
    indices = argsort(angles)
    if clockwise:
        return [cloud[i] for i in indices]
    else:
        return [cloud[i] for i in indices[::-1]]    

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
    filtered_cloud = []
    if solidcloud is not None:
        # Go through the pointcloud
        print("Size of cloud:", solidcloud.shape)
        for i in range(0,solidcloud.shape[0]):
            point = solidcloud[i,:]
            x,y,z = point[0],point[1],point[2]
            # Save to csv (enable the following)
            """
            if not write_once:
                save_cloud(x,y,z)
            """
            # Remove extreme points
            if((x>=x_min)and(x<=x_max))and((y>=y_min)and(y<=y_max))and((z>=z_min)and(z<=z_max)):
                filtered_cloud.append([x,y,z])
        # Save to csv (enable the following)
        # write_once = True 

        # Rearrange the cloud for path generatrion
        start_pt = (0,0)
        path = rotational_sort(filtered_cloud,start_pt)

        # Follow the path
        print("Path Planning start ...")
        for i in range(0,len(path)):
            # Skip points
            if (i%intervals==0):
                print("Path[{}] -> x:{:.2f} y:{:.2f} z:{:.2f}".format(i,x,y,z))
                x,y,z = path[i][0],path[i][1],path[i][2]
                ur.ur3_set_end_effector_goal_quat(x,y,z,ur.init_pose[3],\
                    ur.init_pose[4],ur.init_pose[5],ur.init_pose[6])     


    
        


