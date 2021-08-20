#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void callback(const PointCloud::ConstPtr& msg)
{
    printf("Cloud: width = %d, height = %d\n",msg->width,msg->height);
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"path_planner");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<PointCloud>("/organized_edge_detector/output_occluding_edge",1,callback);
    ros::spin();
}