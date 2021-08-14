# ur3_edge_follower

## JSK PCL Installation
```
sudo apt-get install ros-melodic-jsk-pcl-ros        # Install core package
sudo apt-get install ros-melodic-jsk-rviz-plugins   # Allow to check normal vector
sudo apt-get install ros-melodic-rgbd-launch        # Provide organized pointcloud
```

## Camera Bringup
```
# this invoke the camera to produce Organized point clouds
roslaunch realsense2_camera rs_rgbd.launch camera:=d415   

# this will show you the RViz with all the different edges detected          
roslaunch jsk_pcl_ros sample_organized_edge_detector.launch
```