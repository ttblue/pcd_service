#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <eigen_conversions/eigen_msg.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

//#include <ar_track_service/MarkerPositions.h>

using namespace Eigen;
using namespace std;

typedef Eigen::Matrix<bool,Eigen::Dynamic,1> VectorXb;
typedef pcl::PointXYZRGB ColorPoint;
typedef pcl::PointCloud<ColorPoint> ColorCloud;
typedef ColorCloud::Ptr ColorCloudPtr;

/**
   Finds AR markers and their transforms from point clouds.

bool findARMarkers (ColorCloudPtr cloud, vector<int> &ids, vector<Affine3d> &tfms) {

  pcl::PCLPointCloud2 cloud_PCLPC2;
  pcl::toPCLPointCloud2(*cloud, cloud_PCLPC2);
  sensor_msgs::PointCloud2 ros_pc;
  pcl_conversions::fromPCL(cloud_PCLPC2, ros_pc);

  ar_track_service::MarkerPositions markers;
  markers.request.pc = ros_pc;

  if (client.call(markers)) {
    ids.clear(); tfms.clear();
    ar_track_service::MarkerPositions::Response res = markers.response;
    
    if (res.markers.markers.size() == 0) return false;

    for (int i=0; i<res.markers.markers.size(); ++i) {
      Affine3d tfm;
      tf::poseMsgToEigen(res.markers.markers[i].pose.pose, tfm);

      ids.push_back(res.markers.markers[i].id);
      tfms.push_back(tfm);
    }
    return true;
  }
  else return false;
}


/**
   ids1,tfms1 correspond to the markers found in camera1's frame.
   ids2,tfms2 correspond to the markers found in camera2's frame.

   Finds rel transform from camera1 to camera2. 
   As in, it takes points from camera2's frame and puts them into camera1's frame.
   
   Right now, using 3 as hard coded marker.

bool findRelTransform (vector<int> &ids1, vector<Affine3d> &tfms1,
		       vector<int> &ids2, vector<Affine3d> &tfms2,
		       Affine3d &relTfm) {

  if (ids1.size() ==0 || ids2.size() == 0) {
    cout<<"One of the point clouds do not contain markers."<<endl;
    return false;
  }
  
  // Naive way of finding common markers but idc
  // Using Marker 3 -- hardcoded.
  for (i = 0; i < ids1.size(); ++i) {
    for (j = 0; j < ids2.size(); ++j) {
      if (ids1[i] == ids2[j] && ids1[i] == 3) {
	relTfm = tfms1[i]*tfms2[j].inverse();
	return true;
      }
    }
  }
  return false;
}

bool getTransform(int id, Affine3d &tfm,
		  vector<int> &ids, vector<Affine3d> &tfms) {

  for (i = 0; i<ids.size(); i++)
    if (ids[i] == id) {
      tfm = tfms[i];
      return true;
    }
  return false;
    
  }**/


/**
   Convert point cloud to eigen matrix.
**/
MatrixXf toEigenMatrix(ColorCloudPtr cloud) {
  return cloud->getMatrixXfMap(3,8,0);
}

/**
   Return cloud where mask is true.
**/
ColorCloudPtr maskCloud(const ColorCloudPtr in, const VectorXb& mask) {

  ColorCloudPtr out(new ColorCloud());
  int nOut = mask.sum();
  out->reserve(nOut);
  out->header=in->header;
  out->width = nOut;
  out->height = 1;
  out->is_dense = false;

  int i = 0;
  BOOST_FOREACH(const ColorPoint& pt, in->points) {
    if (mask(i)) out->push_back(pt);
    ++i;
  }

  return out;
}

/**
   Box filter for cloud.
**/
ColorCloudPtr orientedBoxFilter(ColorCloudPtr cloud_in, const Matrix3f& ori, const Vector3f& mins, const Vector3f& maxes) {
  MatrixXf cloudxyz = toEigenMatrix(cloud_in).transpose();
  cout<<"Rows: "<<cloudxyz.rows()<<" and Cols: "<<cloudxyz.cols()<<endl;
  MatrixXf xyz = cloudxyz * ori;
  VectorXb mask(xyz.rows());
  for (int i=0; i < xyz.rows(); i++) {
    mask(i) = (xyz(i,0) >= mins(0)) &&
      (xyz(i,1) >= mins(1)) &&
      (xyz(i,2) >= mins(2)) &&
      (xyz(i,0) <= maxes(0)) &&
      (xyz(i,1) <= maxes(1)) &&
      (xyz(i,2) <= maxes(2));
  }
  ColorCloudPtr cloud_out = maskCloud(cloud_in, mask);
  return cloud_out;
}

/**
   Downsample cloud.
**/
ColorCloudPtr downsampleCloud(const ColorCloudPtr in, float sz) {
  pcl::PointCloud<ColorPoint>::Ptr out(new pcl::PointCloud<ColorPoint>());
  pcl::VoxelGrid<ColorPoint> vg;
  vg.setInputCloud(in);
  vg.setLeafSize(sz,sz,sz);
  vg.filter(*out);
  return out;
}


/**
   Creates and returns a viewer.
 **/
boost::shared_ptr<pcl::visualization::PCLVisualizer> View1 ()  {
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


bool icp_refining(ColorCloudPtr cloud1, ColorCloudPtr cloud2) {

  //Boxfilter the clouds around AR markers.
  Matrix3f eye = Matrix3f::Identity();
  // Cloud 1 -- Hardcoding values to not deal with PCL-1.5
  Vector3f ar3_1(0.185515060929,0.0206483846594, 0.826703742698);
  Vector3f mins1  = ar3_1 - Vector3f(1.0,0.5,0.4);
  Vector3f maxes1  = ar3_1 + Vector3f(0.50,0.4,1.1);
  cloud1 = orientedBoxFilter(cloud1, eye, mins1, maxes1);
  //pcl::io::savePCDFileASCII ("cloud1_filtered.pcd", *cloud1);

  // Cloud 2 -- Hardcoding values to not deal with PCL-1.5
  Vector3f ar3_2(0.175211839792, -0.153883984427, 1.03197535468);
  Vector3f mins2  = ar3_2 - Vector3f(1.0,0.5,0.8);
  Vector3f maxes2  = ar3_2 + Vector3f(0.50,0.6,1.1);
  cloud2 = orientedBoxFilter(cloud2, eye, mins2, maxes2);
  //pcl::io::savePCDFileASCII ("cloud2_filtered.pcd", *cloud2);
  /*
array([[ 0.58331438, -0.54525825,  0.60202805, -0.62187315],
       [ 0.46946875,  0.83117043,  0.29791744, -0.24114774],
       [-0.66282986,  0.10885383,  0.74081537,  0.19508703],
       [ 0.        ,  0.        ,  0.        ,  1.        ]]);*/

  Matrix3f r;
  r.col(0)  = Vector3f(0.58331438, 0.46946875, -0.66282986);
  r.col(1)  = Vector3f(-0.54525825, 0.83117043, 0.10885383);
  r.col(2)  = Vector3f(0.60202805, 0.29791744, 0.74081537);
  Vector3f t(-0.62187315,-0.24114774, 0.19508703);
  
  Matrix4f icp_guess = Matrix4f::Identity();
  icp_guess.block(0,0,3,3) = r;
  icp_guess.block(0,3,3,1) = t;
  //icp_guess.linear() = r; icp_guess.translation() = t;

  //Eigen::Matrix4f icp_guess;
  //icp_guess.block(0,0,3,3) = guess_tfm.linear().cast<float>().transpose();
  //icp_guess.block(0,3,3,1) = guess_tfm.translation().cast<float>();


  pcl::GeneralizedIterativeClosestPoint<ColorPoint, ColorPoint> icp;
  icp.setInputSource(cloud1);
  icp.setInputTarget(cloud2);

  icp.setMaximumIterations(200);
  icp.setRANSACIterations(1000);
  icp.setRANSACOutlierRejectionThreshold(0.05);
  icp.setMaxCorrespondenceDistance(0.05);
  icp.setTransformationEpsilon(1e-10);
  icp.setEuclideanFitnessEpsilon(1e-10);

  std::cout<<"Max iterations: "<<icp.getMaximumIterations ()<<std::endl;
  std::cout<<"Max RANSAC iterations: "<<icp.getRANSACIterations ()<<std::endl;
  std::cout<<"RANSAC Outlier Rejection Threshold: "<< icp.getRANSACOutlierRejectionThreshold ()<<std::endl;
  // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
  std::cout<<"Max correspondence distance: "<<icp.getMaxCorrespondenceDistance ()<<std::endl;
  // Set the transformation epsilon (criterion 2)
  std::cout<<"Transformation epsilon: "<< icp.getTransformationEpsilon ()<<std::endl;
  // Set the euclidean distance difference epsilon (criterion 3)
  std::cout<<"Euclidean fitness epsilon: "<< icp.getEuclideanFitnessEpsilon ()<<std::endl;

  std::cout<<"PC1 size: "<<cloud1->points.size()<<std::endl;
  std::cout<<"PC2 size: "<<cloud2->points.size()<<std::endl;

  ColorCloudPtr Final (new ColorCloud());
  icp.align(*Final, icp_guess);
  std::cout<<"Has converged: "<<icp.hasConverged()<<" with score: "<<icp.getFitnessScore()<<endl;

  cout<<"Initial rel tfm:\n"<<icp_guess<<endl;
  cout<<"ICP rel tfm:\n"<<icp.getFinalTransformation()<<endl;
  
  Matrix4f ntfm = icp.getFinalTransformation().inverse();
  pcl::transformPointCloud(*Final,*Final,ntfm);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = View1();
  pcl::visualization::PointCloudColorHandlerRGBField<ColorPoint> rgb1(cloud1);
  pcl::visualization::PointCloudColorHandlerRGBField<ColorPoint> rgb2(cloud2);
  pcl::visualization::PointCloudColorHandlerRGBField<ColorPoint> rgb3(Final);

  
  //viewer->addPointCloud<ColorPoint> (cloud1,rgb1, "cloud1");
  viewer->addPointCloud<ColorPoint> (cloud2,rgb2, "cloud2");
  viewer->addPointCloud<ColorPoint> (Final,rgb3, "Final");
  while (!viewer->wasStopped()) {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  return true;

}

int main(int argc, char *argv[])
{
  ros::init (argc, argv, "icp_transform");

  ColorCloudPtr cloud_in(new ColorCloud()), cloud_out(new ColorCloud());
  pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/sibi/sandbox/pcd_service/clouds/cloud1.pcd", *cloud_in);  
  pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/sibi/sandbox/pcd_service/clouds/cloud2.pcd", *cloud_out);

  bool done = icp_refining(cloud_in, cloud_out);
  return 0;
}
