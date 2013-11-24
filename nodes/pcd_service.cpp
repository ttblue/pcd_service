#include <iostream>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "pcd_service/PCDData.h"

typedef pcl::PointCloud<pcl::PointXYZRGB> RGBCloud;

/** Saves the two point-clouds as .pcd files and the transform in a text-file.**/
bool saveData (pcd_service::PCDData::Request &req,
	       pcd_service::PCDData::Response &res) {

  // Do ROS --> PCL conversions:
  pcl::PCLPointCloud2 cloud_in_PCLP2;
  RGBCloud::Ptr cloud_in(new RGBCloud);
  pcl_conversions::toPCL(req.pc, cloud_in_PCLP2);
  pcl::fromPCLPointCloud2(cloud_in_PCLP2, *cloud_in);

  // write the point-clouds and transform data to files:
  pcl::io::savePCDFileASCII (req.fname, *cloud_in);
  return true; 
}


int main(int argc, char *argv[]) {
  ros::init (argc, argv, "pcd_saver_service_node");
  ros::NodeHandle n;
  ros::ServiceServer pcdSaverService =
    n.advertiseService ("pcd_service", saveData);

  std::cout<<"Spawned PCD saver service. Ready for request."<<std::endl;
  ros::Duration(1.0).sleep();
  ros::spin ();

  return 0;
}
