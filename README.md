# ur3_edge_follower

## System Preparation
### A. Realsense Installation
1. Install realsense2-camera for ROS Melodic
```
sudo apt install ros-$ROS_DISTRO-librealsense2 ros-$ROS_DISTRO-realsense2-camera ros-$ROS_DISTRO-realsense2-description
```

2. Copy the udev file for realsense-device
```
wget https://github.com/IntelRealSense/librealsense/raw/master/config/99-realsense-libusb.rules
sudo cp 99-realsense-libusb.rules /etc/udev/rules.d/
```

### B. JSK PCL Installation
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

### C.Camera Bringup
You can test the realsense camera and jsk pcl package by the following instructions.

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
# line 13 and line 14
<remap from="~input" to="/d415/depth_registered/points"/>
# YOu need to change this topic if you are using other camera.
```

## Robot Vision Control
1. Bringup the robot
```
./start_robot_bringup.sh
```

2. Start robot pcl planning rviz
```
./start_robot_pcl_planning.sh
```

![image](https://github.com/vincent51689453/ur3_edge_follower/blob/main/git_image/sample2.png)

## Information
### A. ROS related
In order to create a C++ PCL package, you need to perform several actions.
1. Create a ros package
```
# You have to replace your own package name with my_pcl_tutorial
catkin_create_pkg my_pcl_tutorial pcl_conversions pcl_ros roscpp sensor_msgs 
```

2. Modify package.xml and add the followings
```
<build_depend>libpcl-all-dev</build_depend>
<exec_depend>libpcl-all</exec_depend>
```

3. Modify CMakeLists.txt of the package
```
# You have to replace your xxx.cpp with pcl_subscribe.cpp
add_executable(${PROJECT_NAME}_node src/pcl_subscribe.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
```

4. VSCode configuration.
Sometimes your vscode may not able seek for the ros header file. Therefore you can use my .vscode/c_cpp_properties.json to solve this issue.


### B. Reference
1. jsk_pcl_ros/sample/sample_organized_edge_detector.launch

2. jsk_pcl_ros_utils/sample/sample_normal_concatenater.launch

3. jsk_pcl_ros/sample/sample_normal_estimation_omp.launch

4. jsk_pcl_ros/sample/sample_line_segment_detector.launch


