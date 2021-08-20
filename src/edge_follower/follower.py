from __future__ import division
import numpy as np
import ur_robot as ur
import time

cloud_store = False
cloudXYZ = None

start_pt_flag = False
start_pt = None         # Starting point of follower
start_index = 0         # Starting point index of the pointcloud

path = []               # The entire path
path_x_max = -2         # x_max of cloud


# Get the i-th point XYZ data from an organized cloud
def get_cloud_point(cloud,i):
    return cloud[i,:]

# Transform the point unit 
def point_unit_transform(list_xyz,scale):
    x = list_xyz[0]/scale
    y = list_xyz[1]/scale
    z = list_xyz[2]/scale
    return [x,y,z]

# It will guide the robot and follow the point cloud
def guide(cloud_np,ready):
    global cloud_store,cloudXYZ
    global start_pt,start_pt_flag,start_index
    xyz = [-10,-10,-10]
    rpy = [0,0,0]
    # Make sure the robot is in ready pose
    if ready:
        if not cloud_store:
            print("Start pcl detection...")
            time.sleep(10)
            # Save the cloud
            cloudXYZ = cloud_np
            print("CloudXYZ is stored...")
            print("Shape of the cloudXYZ:{}".format(cloud_np.shape))
            cloud_store = True
            #print(cloudXYZ)
        else:
            # Follow the cloud
            # Ignore some points and find the correct starting point
            i = 0       
            start_index = i
            xyz = cloudXYZ[500,:]
            xyz = point_unit_transform(xyz,1)
            start_pt = xyz

            print("Target:{} Index:{}".format(start_pt,start_index))

            # Drive the robot
            ur.ur3_set_end_effector_goal_quat(start_pt[0],start_pt[1],0.2,ur.init_pose[3],\
                    ur.init_pose[4],ur.init_pose[5],ur.init_pose[6])
            time.sleep(1000)
   
    return xyz,rpy

    
