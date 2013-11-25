#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>


class visualizer {
  /** A simple class to help visualize the PCL point clouds.*/
public: 
    /** A PCL viewer instance.*/
  pcl::visualization::CloudViewer viewer;

  /** Initialize the viewer with the name as WINDOWNAME.*/
  visualizer(std::string *windowName =  new std::string("visualizer")) 
  : viewer(*windowName) { }

  /** Display the point_cloud CLOUD till the user exits.
      Holds the thread busy.*/
  void display(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) {
    viewer.showCloud(cloud);
  }

  /** Returns TRUE iff the gui of the viewer was stopped.*/
  bool wasStopped() {return viewer.wasStopped();}
};
visualizer viz;


void load_and_view(std::string fname) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::io::loadPCDFile<pcl::PointXYZRGB>(fname, *cloud);  
  viz.display(cloud);
}


int main(int argc, char** argv) {
  if (argc < 2)
    std::cout << "Usage: \n\t ./view_pcd <pcd-file-name>\n";
  std::cout << "Showing .pcd file: "<<argv[1]<<std::endl;
  load_and_view(argv[1]);
  while(1){}
}
