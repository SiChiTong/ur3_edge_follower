# ur3_edge_follower

## Realsense Installation
1. Install realsense2-camera for ROS Melodic
```
sudo apt install ros-$ROS_DISTRO-librealsense2 ros-$ROS_DISTRO-realsense2-camera ros-$ROS_DISTRO-realsense2-description
```

2. Copy the udev file for realsense-device
```
wget https://github.com/IntelRealSense/librealsense/raw/master/config/99-realsense-libusb.rules
sudo cp 99-realsense-libusb.rules /etc/udev/rules.d/
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

You may encounter an error about fail to load a .bag file. You need to go to /opt/ros/melodic/share/jsk_pcl_ros/sample/sample_organzied_edge_detector.launch and comment out line 4 to fix this issue. That line is about to load a play_rosbag_shelf_bin.xml.

3. Edit the launch file by changing the input topic to your camera's pointcloud.
```
<remap from="~input" to="/d415/depth_registered/points"/>
# YOu need to change this topic if you are using other camera.
```


