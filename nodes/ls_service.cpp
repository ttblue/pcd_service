#include <iostream>
#include <assert.h>
#include <math.h>

#include <Eigen/SVD>

#include <pcl/point_types.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/median_filter.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl_conversions/pcl_conversions.h>
#include <eigen_conversions/eigen_msg.h>

#include "icp_service/LSTransform.h"

using namespace std;
using namespace Eigen;

typedef Eigen::Matrix<bool,Eigen::Dynamic,1> VectorXb;
typedef pcl::PointXYZRGB ColorPoint;
typedef pcl::PointCloud<ColorPoint> ColorCloud;
typedef ColorCloud::Ptr ColorCloudPtr;
typedef pcl::visualization::PointCloudColorHandlerCustom<ColorPoint> ColorHandlerT;


/*
  Gives transform from frame of points 1 to frame of points 2.
  Which means, applying the transform to points2 will make them in 
  points1' frame.
*/
Matrix4f findRigidTfm (MatrixXf points1, MatrixXf points2) {
    
  assert (("Incorrect sizes.", points1.rows() == points2.rows() || points1.cols() == points2.cols()));
  assert (("Too few points.", points1.rows() >= 3));
    
  size_t N = points1.rows();
  RowVector3f center1 = points1.colwise().sum()/N;
  RowVector3f center2 = points2.colwise().sum()/N;

  // Want to go from 1 to 2
  MatrixXf Y = (points1.rowwise() - center1);
  MatrixXf X = (points2.rowwise() - center2);
  
  MatrixXf S = X.transpose()*Y;
  JacobiSVD <MatrixXf> svd(S, ComputeFullU | ComputeFullV);
 
  Matrix3f ref_rot = Matrix3f::Identity();
  ref_rot(2,2) = round((svd.matrixV()*svd.matrixU().transpose()).determinant());

  Matrix4f finalTfm = Matrix4f::Identity();
  Matrix3f R = svd.matrixV()*ref_rot*svd.matrixU().transpose();
  finalTfm.block(0,0,3,3) = R;
  finalTfm.block(0,3,3,1) = center1.transpose() - R*center2.transpose();
    
  return finalTfm;
}


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
ColorCloudPtr AABBFilter(ColorCloudPtr cloud_in, const Vector3f& mins, const Vector3f& maxes) {

  // Axis aligned.
  Matrix3f ori = Matrix3f::Identity();

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

/*
Use RANSAC to fit a plane and then remove it.
 */
ColorCloudPtr discardPlanePoints (ColorCloudPtr cloud_in) {
  
  std::vector<int> inlier_indices;

  // created RandomSampleConsensus object and compute the appropriated model
  pcl::SampleConsensusModelPlane<ColorPoint>::Ptr
    model_p (new pcl::SampleConsensusModelPlane<ColorPoint> (cloud_in));

  ColorCloudPtr cloud_out (new ColorCloud());

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

  return cloud_out;
}

ColorCloudPtr medianFilter (ColorCloudPtr cloud_in) {
  pcl::MedianFilter<ColorPoint> medf;
  ColorCloudPtr cloud_out(new ColorCloud());

  medf.setWindowSize(5);
  medf.setMaxAllowedMovement(0.001);
  medf.setInputCloud (cloud_in);
  medf.filter (*cloud_out);

  return cloud_out;
}

/*
  Does some hard coding for extents of tables.
*/
void minsMaxes (Matrix4f arPose, float w, float l, 
		Vector3f& mins, Vector3f& maxes) {

  Vector3f x = arPose.block(0,0,3,1);
  Vector3f y = arPose.block(0,1,3,1);
  Vector3f z = arPose.block(0,2,3,1);
  Vector3f t = arPose.block(0,3,3,1);
  
  // Don't bother with relation between x,y and w,l
  float m = (w > l) ? w : l;
  // Arbitrary number for scale.
  // 10 cm for below table
  float scale = 0.75;
  mins = t - ((x+y)*m*scale)-z*0.1;
  maxes = t + ((x+y)*m*scale)+z*0.4;
}

/*
  Finds least squares transform that transforms cloud2 points into
  cloud1's frame.
  Find better way to do kdtree search?
 */
Matrix4f fitLSTransform(ColorCloudPtr cloud1, ColorCloudPtr cloud2,
			Matrix4f init_tfm = Matrix4f::Identity()) {

  
  float xtol = 1e-2;
  float ftol = 1e-1;
  int MAX_ITER = 30;
  int iter = 1;
  float sz = 0.005;

  // KDTree being used.
  pcl::KdTreeFLANN<ColorPoint> kdtree;
  kdtree.setInputCloud (cloud1);
  //ColorCloudPtr cloud2_ds = downsampleCloud(cloud2, sz);

  Matrix4f finalTfm = init_tfm;
  pcl::transformPointCloud(*cloud2_ds, *cloud2_ds, init_tfm);
  
  // Main loop for fitting transform.
  while (iter <= MAX_ITER) {
    cout << "Iteration: "<< i<<endl;
    float distTol = max(0.5 -  4*i/30, 0.05);
    
    cout << "Finding nearest points."<<endl;
    float distTot = 0.0;
    // Fill in new clouds with nearest neighbors.
    // Could directly do this with MatrixXf maybe.
    ColorCloudPtr cloud1_n (new ColorCloud()), cloud2_n (new ColorCloud());
    BOOST_FOREACH(const ColorPoint& pt, cloud2_ds->points) {
      vector< int > k_indices;
      vector< float > k_sqr_distances;
      if (kdtree.radiusSearch(pt, distTol, k_indices, k_sqr_distances, 1) > 0) {
	cloud1_n->push_back(cloud1->points[k_indices[0]]);
	cloud2_n->push_back(pt);
	distTot += sqrt(k_sqr_distances[0]);
      }
    }

    if (distTot < ftol) {
      cout <<"Tolerance threshold reached."<< endl;
      break;
    }

    MatrixXf points1 = toEigenMatrix(*cloud1_n);
    MatrixXf points2 = toEigenMatrix(*cloud2_n);

    Matrix4f tfm = findRigidTfm (points1, points2);
    pcl::transformPointCloud(*cloud2_ds, *cloud2_ds, tfm);
    finalTfm = tfm*finalTfm;

    iter ++;
  }
  
  if (iter > MAX_ITER)
    cout << "Reached maximum number of iterations." << endl;

  return finalTfm;
}


bool findLSTransform(icp_service::LSTransform::Request &req, 
		     icp_service::LSTransform::Response &res) {

  // Relevant constants.
  float w = 0.85, l = 1.35;

  // Convert to pcl point clouds from ROS types
  pcl::PCLPointCloud2 cloud1_PCLPC2, cloud2_PCLPC2;
  ColorCloud::Ptr cloud1(new ColorCloud);
  ColorCloud::Ptr cloud2(new ColorCloud);

  pcl_conversions::toPCL(req.pc1, cloud1_PCLPC2);
  pcl_conversions::toPCL(req.pc2, cloud2_PCLPC2);

  pcl::fromPCLPointCloud2(cloud1_PCLP2, *cloud1);
  pcl::fromPCLPointCloud2(cloud2_PCLP2, *cloud2);
  
  // Get AR and guess transforms 
  Eigen::Affine3d guess_tfm, ar_tfm1, ar_tfm2;
  tf::poseMsgToEigen(req.guess,guess_tfm);
  tf::poseMsgToEigen(req.arPose1,ar_tfm1);
  tf::poseMsgToEigen(req.arPose2,ar_tfm2);

  Eigen::Matrix4f icp_guess;
  icp_guess.block(0,0,3,3) = guess_tfm.linear().cast<float>();
  icp_guess.block(0,3,3,1) = guess_tfm.translation().cast<float>();
  Eigen::Matrix4f ar_hmat1, ar_hmat2;
  ar_hmat1.block(0,0,3,3) = ar_tfm1.linear().cast<float>();
  ar_hmat1.block(0,3,3,1) = ar_tfm1.translation().cast<float>();
  ar_hmat2.block(0,0,3,3) = ar_tfm2.linear().cast<float>();
  ar_hmat2.block(0,3,3,1) = ar_tfm2.translation().cast<float>();

  // Find mins and maxes of bounding box for box filter.
  Vector3f mins1, maxes1, mins2, maxes2;
  minsMaxes (ar_hmat1, w, l, mins1, maxes1);
  minsMaxes (ar_hmat2, w, l, mins2, maxes2);

  // Filter out everything but the table.
  cloud1 = AABBFilter(cloud1, mins1, maxes1);
  cloud1 = discardPlanePoints(cloud1);
  cloud2 = AABBFilter(cloud2, mins2, maxes2);
  cloud2 = discardPlanePoints(cloud2);

  // Transform cloud2 to cloud1's frame;
  pcl::transformPointCloud (*cloud2, *cloud2, icp_guess);

  // Find least squares transform.
  Matrix4f finalTfm = fitLSTransform (cloud1, cloud2);

  Affine3d found_tfm;
  found_tfm.linear() = finalTfm.cast<double>().block(0,0,3,3);
  found_tfm.translation() = finalTfm.cast<double>().block(0,3,3,1);;
  tf::poseEigenToMsg(found_tfm, res.pose);

  return true; 

}

int main(int argc, char *argv[])
{
  ros::init (argc, argv, "ls_transform");
  ros::NodeHandle n;
  ros::ServiceServer LSService =
    n.advertiseService ("LSTransform", findLSTransform);

  std::cout<<"Spawned LS service. Ready for request."<<std::endl;

  ros::Duration(1.0).sleep();
  ros::spin ();

  return 0;
}
