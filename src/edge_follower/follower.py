from __future__ import division
import rospy
import numpy as np
import tf
import csv
import math
from scipy.signal import savgol_filter
import ur_robot as ur

saved_cloudXYZ = None
cloud_csv = './src/edge_follower/save_cloud.csv'
write_once = False
intervals = 1

# Sorting
refvec = [0,1]
origin = [0,0]

# Seperate the cloud to X/Y/Z channel
def seperate_cloud_channel(cloud):
    ch_x,ch_y,ch_z = [],[],[]
    for i in range(0,len(cloud)):
        ch_x.append(cloud[i][0])
        ch_y.append(cloud[i][1])
        ch_z.append(cloud[i][2])
    return ch_x,ch_y,ch_z

# Sorting operations
def clockwiseangle_and_distance(point):
    global origin,refvec
    # Vector between point and the origin: v = p - o
    vector = [point[0]-origin[0], point[1]-origin[1]]
    # Length of vector: ||v||
    lenvector = math.hypot(vector[0], vector[1])
    # If length is zero there is no angle
    if lenvector == 0:
        return -math.pi, 0
    # Normalize vector: v/||v||
    normalized = [vector[0]/lenvector, vector[1]/lenvector]
    dotprod  = normalized[0]*refvec[0] + normalized[1]*refvec[1]     # x1*x2 + y1*y2
    diffprod = refvec[1]*normalized[0] - refvec[0]*normalized[1]     # x1*y2 - y1*x2
    angle = math.atan2(diffprod, dotprod)
    # Negative angles represent counter-clockwise angles so we need to subtract them 
    # from 2*pi (360 degrees)
    if angle < 0:
        return 2*math.pi+angle, lenvector
    # I return first the angle because that's the primary sorting criterium
    # but if two vectors have the same angle then the shorter distance should come first.
    return angle, lenvector

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

        # Maxima/Minima suppression
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


        global origin,refvec
        # Sort the pointcloud with clockwise
        origin[0] = filtered_cloud[0][0]
        origin[1] = filtered_cloud[0][1]        
        cloudXYZ_sort = sorted(filtered_cloud,key=clockwiseangle_and_distance)

        # Smooth the pointcloud
        i,j,k = seperate_cloud_channel(cloudXYZ_sort)
        window_size = 201
        polynormial_order = 2
        cloudXYZ_smooth = savgol_filter((i,j,k),window_size,polynormial_order)
        point_size = cloudXYZ_smooth.shape
        # Follow the resultant pointcloud
        print("Start path planning ...")
        for i in range(0,point_size[1]):
            goal_x,goal_y,goal_z = cloudXYZ_smooth[0][i],cloudXYZ_smooth[1][i],cloudXYZ_smooth[2][i]
            # Robot control
            print("Path {}: x:{} y:{} z:{}".format(i,goal_x,goal_y,goal_z))
            ur.ur3_set_end_effector_goal_quat(goal_x,goal_y,goal_z,ur.init_pose[3],\
                ur.init_pose[4],ur.init_pose[5],ur.init_pose[6])   


    
        


