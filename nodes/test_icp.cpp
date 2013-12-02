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
#include <pcl/sample_consensus/ransac.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/median_filter.h>

//#include <ar_track_service/MarkerPositions.h>

using namespace Eigen;
using namespace std;

typedef Eigen::Matrix<bool,Eigen::Dynamic,1> VectorXb;
typedef pcl::PointXYZRGB ColorPoint;
typedef pcl::PointXYZRGBNormal ColorPointNormal;
typedef pcl::PointCloud<ColorPoint> ColorCloud;
typedef pcl::PointCloud<ColorPointNormal> ColorNormalCloud;
typedef ColorCloud::Ptr ColorCloudPtr;
typedef ColorNormalCloud::Ptr ColorNormalCloudPtr;
typedef pcl::registration::TransformationEstimationPointToPlaneLLS<ColorPointNormal, ColorPointNormal> PointToPlane;
typedef pcl::visualization::PointCloudColorHandlerCustom<ColorPoint> ColorHandlerT;
typedef pcl::visualization::PointCloudColorHandlerCustom<ColorPointNormal> ColorHandlerNT;

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


void normalEstimation(ColorCloudPtr& cloud_in, ColorNormalCloudPtr& cloud_out)
{
  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<ColorPoint, ColorPointNormal> ne;
  ne.setInputCloud (cloud_in);
  pcl::search::KdTree<ColorPoint>::Ptr tree (new pcl::search::KdTree<ColorPoint> ());
  ne.setSearchMethod (tree);
  ne.setRadiusSearch (0.05);
  ne.compute (*cloud_out);
  pcl::copyPointCloud (*cloud_in, *cloud_out);
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

void discard_plane_points (ColorCloudPtr &cloud_in, ColorCloudPtr &cloud_out) {
  
  std::vector<int> inlier_indices;

  // created RandomSampleConsensus object and compute the appropriated model
  pcl::SampleConsensusModelPlane<ColorPoint>::Ptr
    model_p (new pcl::SampleConsensusModelPlane<ColorPoint> (cloud_in));


  pcl::RandomSampleConsensus<ColorPoint> ransac (model_p);
  ransac.setDistanceThreshold (.02);
  ransac.computeModel();
  ransac.getInliers(inlier_indices);

  // get the points which do not lie on a plane:
  pcl::ExtractIndices<ColorPoint> extract;
  extract.setInputCloud (cloud_in);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  inliers->indices = inlier_indices;
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*cloud_out);

}

void median_filter (ColorCloudPtr &cloud_in, ColorCloudPtr &cloud_out) {
  pcl::MedianFilter<ColorPoint> medf;
  medf.setWindowSize(5);
  medf.setMaxAllowedMovement(0.001);
  medf.setInputCloud (cloud_in);
  medf.filter (*cloud_out);
}

bool icp_refining(ColorCloudPtr cloud1_orig, ColorCloudPtr cloud2_orig) {

  median_filter(cloud1_orig, cloud1_orig);
  median_filter(cloud2_orig, cloud2_orig);

  ColorCloudPtr cloud1 (new ColorCloud()), cloud2 (new ColorCloud());

  //Boxfilter the clouds around AR markers.
  Matrix3f eye = Matrix3f::Identity();
  // Cloud 1 -- Hardcoding values to not deal with PCL-1.5
  Vector3f ar3_1(0.185515060929,0.0206483846594, 0.826703742698);
  Vector3f mins1  = ar3_1 - Vector3f(1.0,0.5,0.4);
  Vector3f maxes1  = ar3_1 + Vector3f(0.50,0.4,1.1);
  cloud1_orig = orientedBoxFilter(cloud1_orig, eye, mins1, maxes1);
  discard_plane_points(cloud1_orig, cloud1);
  pcl::io::savePCDFileASCII ("cloud1_mf_noplane.pcd", *cloud1);
  //cloud1 =  downsampleCloud(cloud1, 0.005);

  //pcl::io::savePCDFileASCII ("cloud1_filtered.pcd", *cloud1);

  // Cloud 2 -- Hardcoding values to not deal with PCL-1.5
  Vector3f ar3_2(0.175211839792, -0.153883984427, 1.03197535468);
  Vector3f mins2  = ar3_2 - Vector3f(1.0,0.5,0.8);
  Vector3f maxes2  = ar3_2 + Vector3f(0.50,0.6,1.1);
  cloud2_orig = orientedBoxFilter(cloud2_orig, eye, mins2, maxes2);
  discard_plane_points(cloud2_orig, cloud2);
  //cloud2 =  downsampleCloud(cloud2, 0.005);
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
  pcl::transformPointCloud(*cloud2,*cloud2,icp_guess);
  //pcl::io::savePCDFileASCII ("cloud2_mf_noplane.pcd", *cloud2);
  //pcl::io::savePCDFileASCII ("cloud2_filtered_transformed.pcd", *cloud2);

  //Eigen::Matrix4f icp_guess;
  //icp_guess.block(0,0,3,3) = guess_tfm.linear().cast<float>().transpose();
  //icp_guess.block(0,3,3,1) = guess_tfm.translation().cast<float>();

  ColorNormalCloudPtr cloudn1 (new ColorNormalCloud ()), cloudn2 (new ColorNormalCloud ());
  cout<<"Finding normals for pc1."<<endl;
  //normalEstimation(cloud1, cloudn1);
  cout<<"Finding normals for pc2."<<endl;
  //normalEstimation(cloud2, cloudn2);

  cout<<"Setting up ICP."<<endl;

  /*  pcl::GeneralizedIterativeClosestPoint<ColorPointNormal, ColorPointNormal> icp;
  icp.setInputSource(cloudn1);
  icp.setInputTarget(cloudn2);

  boost::shared_ptr<PointToPlane> point_to_plane(new PointToPlane());
  icp.setTransformationEstimation(point_to_plane);

  icp.setMaximumIterations(300);
  icp.setRANSACIterations(1000);
  icp.setRANSACOutlierRejectionThreshold(0.05);
  icp.setMaxCorrespondenceDistance(0.05);
  icp.setTransformationEpsilon(1e-8);
  icp.setEuclideanFitnessEpsilon(1e-8);

  std::cout<<"Max iterations: "<<icp.getMaximumIterations ()<<std::endl;
  std::cout<<"Max RANSAC iterations: "<<icp.getRANSACIterations ()<<std::endl;
  std::cout<<"RANSAC Outlier Rejection Threshold: "<< icp.getRANSACOutlierRejectionThreshold ()<<std::endl;
  // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
  std::cout<<"Max correspondence distance: "<<icp.getMaxCorrespondenceDistance ()<<std::endl;
  // Set the transformation epsilon (criterion 2)
  std::cout<<"Transformation epsilon: "<< icp.getTransformationEpsilon ()<<std::endl;
  // Set the euclidean distance difference epsilon (criterion 3)
  std::cout<<"Euclidean fitness epsilon: "<< icp.getEuclideanFitnessEpsilon()<<std::endl;*/

  //std::cout<<"PC1 size: "<<cloudn1->points.size()<<std::endl;
  //std::cout<<"PC2 size: "<<cloudn2->points.size()<<std::endl;

  ColorNormalCloudPtr Final (new ColorNormalCloud());
  //icp.align(*Final, Matrix4f::Identity());
  //std::cout<<"Has converged: "<<icp.hasConverged()<<" with score: "<<icp.getFitnessScore()<<endl;

  cout<<"Initial rel tfm:\n"<<icp_guess<<endl;
  //cout<<"ICP rel tfm:\n"<<icp.getFinalTransformation()<<endl;

  Matrix4f tfm2 = Matrix4f::Identity();
  
  tfm2.row(0)  = Vector4f(9.99991224e-01,   4.14990513e-03,   5.73805956e-04,   6.51068157e-03);
  tfm2.row(1)  = Vector4f(-4.15057632e-03,   9.99990698e-01,   1.17350020e-03,   3.24235007e-03);
  tfm2.row(2)  = Vector4f(-5.68930704e-04,  -1.17587153e-03,   9.99999147e-01,  -3.94321928e-03);
  /*
  T_h_k: [[  9.99991224e-01   4.14990513e-03   5.73805956e-04   6.51068157e-03]
	  [ -4.15057632e-03   9.99990698e-01   1.17350020e-03   3.24235007e-03]
	  [ -5.68930704e-04  -1.17587153e-03   9.99999147e-01  -3.94321928e-03]
	  [  0.00000000e+00   0.00000000e+00   0.00000000e+00   1.00000000e+00]]*/
  pcl::transformPointCloud(*cloud2,*cloud2,tfm2);
  pcl::transformPointCloud(*cloud2_orig,*cloud2_orig,tfm2*icp_guess);
  
  //Matrix4f ntfm = icp.getFinalTransformation().inverse();
  //pcl::transformPointCloud(*cloud2,*cloud2,icp.getFinalTransformation());

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = View1();
  pcl::visualization::PointCloudColorHandlerRGBField<ColorPoint> rgb1(cloud1_orig);
  pcl::visualization::PointCloudColorHandlerRGBField<ColorPoint> rgb2(cloud2_orig);
  //pcl::visualization::PointCloudColorHandlerRGBField<ColorPointNormal> rgb3(Final);

  
  //viewer->addPointCloud<ColorPoint> (cloud1, ColorHandlerT(cloud1, 0.,0.,255.), "cloud1");
  viewer->addPointCloud<ColorPoint> (cloud1_orig,rgb1, "cloud1");
  //viewer->addPointCloud<ColorPoint> (cloud2, ColorHandlerT(cloud2, 255.,0.,0.), "cloud2");
  viewer->addPointCloud<ColorPoint> (cloud2_orig,rgb2, "cloud2");
  //viewer->addPointCloud<ColorPointNormal> (Final,rgb3, "Final");
  //viewer->addPointCloud<ColorPointNormal> (Final,ColorHandlerNT(Final, 0.,0.,255.), "Final");
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
