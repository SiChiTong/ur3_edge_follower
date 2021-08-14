# ur3_edge_follower

## Realsense Installation
1. Register the public server key
```
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
```

2. Add the server to the list of repositories
```
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
```

3. Install the libraries
```
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
```

## JSK PCL Installation
1. Install the core JSK-PCL package
```
sudo apt-get install ros-melodic-jsk-pcl-ros
```

2. Allow the system to check the normal vector
```
sudo apt-get install ros-melodic-jsk-rviz-plugins
```

3. Provide organized pointcloud from the camera
```
sudo apt-get install ros-melodic-rgbd-launch
```

## Camera Bringup
1. This invoke the camera to produce Organized point clouds
```
roslaunch realsense2_camera rs_rgbd.launch camera:=d415   
```

2. This will show you the RViz with all the different edges detected      
```  
roslaunch jsk_pcl_ros sample_organized_edge_detector.launch
```