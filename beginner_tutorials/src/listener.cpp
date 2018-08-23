#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include "beginner_tutorials/marker_6dof.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <boost/thread/thread.hpp>
#include <Eigen/Dense>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string.h>
#include <sstream>
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
using namespace sensor_msgs;
using namespace message_filters;
using namespace cv;
/*pcl::visualization::CloudViewer viewer("Cloud Viewer");
int user_data;

void viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (1.0, 0.5, 1.0);
    pcl::PointXYZ o;
    o.x = 1.0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere (o, 0.25, "sphere", 0);
    std::cout << "i only run once" << std::endl;    
}
void viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
{
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape ("text", 0);
    viewer.addText (ss.str(), 200, 300, "text", 0);
    
    //FIXME: possible race condition here:
    user_data++;
}*/

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  ROS_INFO_STREAM("Velodyne scan received at " << msg->header.stamp.toSec());
}

void markerCallback(const beginner_tutorials::marker_6dof::ConstPtr& msg_rt)
{
  ROS_INFO_STREAM("marker_6dof received at " << msg_rt->header.stamp.toSec());
}

void callback(const sensor_msgs::PointCloud2::ConstPtr& msg_pc, const sensor_msgs::Image::ConstPtr& msg_rt){
  static int count = 1;
  ROS_INFO_STREAM("Velodyne scan received at " << msg_pc->header.stamp.toSec());
  ROS_INFO_STREAM("marker_6dof received at " << msg_rt->header.stamp.toSec());
  pcl::PointCloud<pcl::PointXYZ> point_cloud;
  pcl::PointCloud<pcl::PointXYZ> new_cloud;
  pcl::fromROSMsg(*msg_pc, point_cloud);
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  Eigen::Affine3f lidarToCamera = pcl::getTransformation(0, 0, 0, 0.127308, -3.10387, 1.4642);
  pcl::transformPointCloud(point_cloud, new_cloud, lidarToCamera);
  std::stringstream pcdName;
  std::stringstream imgName;
  pcdName<<"/media/wqyang/Seagate Expansion Drive/vloam/export_file/pointCloud" << count<<".pcd";
  imgName<<"/media/wqyang/Seagate Expansion Drive/vloam/export_file/image" << count<<".jpg";  
  pcl::io::savePCDFileASCII (pcdName.str(), point_cloud);
    
  cv_bridge::CvImagePtr cv_ptr= cv_bridge::toCvCopy(msg_rt,sensor_msgs::image_encodings::BGR8);  
  Mat imgShowMat;
  cv_ptr->image.copyTo(imgShowMat);
  imwrite( imgName.str(), imgShowMat );
  count++;
  

  //*cloudV = new_cloud;
  /*
  viewer.runOnVisualizationThreadOnce (viewerOneOff);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (&new_cloud);
  viewer.showCloud(cloud);
  viewer.runOnVisualizationThread (viewerPsycho);*/
}


int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  //ros::Punlisher pub = projectedHandle.advertise<pcl::PointXYZ> ("projectedPoints", 1);
 
  //ros::Subscriber sub = n.subscribe("/pandar_points", 10, pointCloudCallback);
  //ros::Subscriber sub = n.subscribe("/lidar_camera_calibration_rt", 10, markerCallback);
  
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(n, "/pandar_points", 1);
  //message_filters::Subscriber<beginner_tutorials::marker_6dof> rt_sub(n, "/lidar_camera_calibration_rt", 1);
  message_filters::Subscriber<sensor_msgs::Image> image_sub(n, "/image_publisher_1534837550466394693/image_raw", 1);
  typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), cloud_sub, image_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));
  ROS_INFO("sync, ok");
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
