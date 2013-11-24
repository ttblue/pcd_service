#include <iostream>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> RGBCloud;

int main(int argc, char *argv[]) {

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();

  bool found_cloud = false;
  for (int i = 1; i<argc; ++i) {
    RGBCloud::Ptr cloud(new RGBCloud());
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (std::string(argv[i]), *cloud) == -1)
      {
	PCL_ERROR ("Couldn't read file %s!\n", argv[i]);
      }
    else {
      std::stringstream st; st << "cloud"; st << i;
      viewer->addPointCloud<pcl::PointXYZRGB> (cloud, st.str());
      found_cloud = true;
    }
  }
  if (!found_cloud) return -1;

  while (!viewer->wasStopped ())
    viewer->spinOnce (100);

  return 0;
}
