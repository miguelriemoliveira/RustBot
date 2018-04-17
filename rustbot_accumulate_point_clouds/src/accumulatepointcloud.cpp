//Includes
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/harris_6d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_validation.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/pfh.h>


//Definitions
typedef pcl::PointXYZRGB PointT;
//typedef pcl::PointNormal PS;

//Global vars
std::string filename  = "/tmp/output.pcd";
//std::string filename2 = "/tmp/output.ply";
//std::string filename3 = "/tmp/output_plus_normals.ply";
pcl::PointCloud<PointT>::Ptr accumulated_cloud;
tf::TransformListener *p_listener;
boost::shared_ptr<ros::Publisher> pub;

// Para gravar em PLY
//pcl::PointCloud<PS> output;

pcl::PointCloud<PointT>::Ptr filter_color(pcl::PointCloud<PointT>::Ptr cloud_in){

  // Try to clear white and blue points from sky, and green ones from the grass, or something close to it
  int rMax = 200;
  int rMin = 0;
  int gMax = 120;
  int gMin = 0;
  int bMax = 120;
  int bMin = 0;

  pcl::ConditionAnd<PointT>::Ptr color_cond (new pcl::ConditionAnd<PointT> ());
  color_cond->addComparison (pcl::PackedRGBComparison<PointT>::Ptr (new pcl::PackedRGBComparison<PointT> ("r", pcl::ComparisonOps::LT, rMax)));
  color_cond->addComparison (pcl::PackedRGBComparison<PointT>::Ptr (new pcl::PackedRGBComparison<PointT> ("r", pcl::ComparisonOps::GT, rMin)));
  color_cond->addComparison (pcl::PackedRGBComparison<PointT>::Ptr (new pcl::PackedRGBComparison<PointT> ("g", pcl::ComparisonOps::LT, gMax)));
  color_cond->addComparison (pcl::PackedRGBComparison<PointT>::Ptr (new pcl::PackedRGBComparison<PointT> ("g", pcl::ComparisonOps::GT, gMin)));
  color_cond->addComparison (pcl::PackedRGBComparison<PointT>::Ptr (new pcl::PackedRGBComparison<PointT> ("b", pcl::ComparisonOps::LT, bMax)));
  color_cond->addComparison (pcl::PackedRGBComparison<PointT>::Ptr (new pcl::PackedRGBComparison<PointT> ("b", pcl::ComparisonOps::GT, bMin)));

  // build the filter
  pcl::ConditionalRemoval<PointT> condrem (color_cond);
  condrem.setInputCloud (cloud_in);
  condrem.setKeepOrganized(true);

  // apply filter
  condrem.filter (*cloud_in);

  return cloud_in;

}

pcl::PointCloud<pcl::PointNormal>::Ptr calculate_pointnormals(pcl::PointCloud<PointT>::Ptr cloud_in){

  ROS_INFO("Extraindo coordenadas...");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_ptr (new pcl::PointCloud<pcl::PointXYZ>); // Gather only pose so can compute normals
  cloud_xyz_ptr->resize(cloud_in->points.size()); // allocate memory space
  for(int i=0; i < cloud_in->points.size(); i++){
    cloud_xyz_ptr->points[i].x = cloud_in->points[i].x;
    cloud_xyz_ptr->points[i].y = cloud_in->points[i].y;
    cloud_xyz_ptr->points[i].z = cloud_in->points[i].z;
  }
  ROS_INFO("Calculando normais.......");
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_n(new pcl::search::KdTree<pcl::PointXYZ>());

  ne.setInputCloud(cloud_xyz_ptr);
  ne.setSearchMethod(tree_n);
  ne.setRadiusSearch(0.9);
  ne.compute(*cloud_normals);
  ROS_INFO("Retornando nuvem com normais calculadas!");

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_pointnormals (new pcl::PointCloud<pcl::PointNormal>);
  cloud_pointnormals->resize(cloud_normals->points.size());
  pcl::concatenateFields(*cloud_xyz_ptr, *cloud_normals, *cloud_pointnormals);

  return cloud_pointnormals;

}

pcl::PointCloud<pcl::Normal>::Ptr calculate_normals(pcl::PointCloud<PointT>::Ptr cloud_in){

  ROS_INFO("Extraindo coordenadas...");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_ptr (new pcl::PointCloud<pcl::PointXYZ>); // Gather only pose so can compute normals
  cloud_xyz_ptr->resize(cloud_in->points.size()); // allocate memory space
  for(int i=0; i < cloud_in->points.size(); i++){
    cloud_xyz_ptr->points[i].x = cloud_in->points[i].x;
    cloud_xyz_ptr->points[i].y = cloud_in->points[i].y;
    cloud_xyz_ptr->points[i].z = cloud_in->points[i].z;
  }
  ROS_INFO("Calculando normais.......");
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_n(new pcl::search::KdTree<pcl::PointXYZ>());

  ne.setInputCloud(cloud_xyz_ptr);
  ne.setSearchMethod(tree_n);
  ne.setRadiusSearch(0.9);
  ne.compute(*cloud_normals);
  ROS_INFO("Retornando nuvem com normais calculadas!");

  return cloud_normals;

}

pcl::PointCloud<pcl::PointWithScale> calculate_sift(pcl::PointCloud<pcl::PointNormal>::Ptr normals){

  // Parameters for sift computation
  const float min_scale = 0.01f;
  const int n_octaves = 3;
  const int n_scales_per_octave = 4;
  const float min_contrast = 0.001f;
  ROS_INFO("Comecando calculo SIFT....");
  // Estimate the sift interest points using normals values from xyz as the Intensity variants
  pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointWithScale> sift;
  pcl::PointCloud<pcl::PointWithScale> result;
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal> ());
  sift.setSearchMethod(tree);
  sift.setScales(min_scale, n_octaves, n_scales_per_octave);
  sift.setMinimumContrast(min_contrast);
  sift.setInputCloud(normals);
  sift.compute(result);

  ROS_INFO("SIFT calculado com %d pontos!", result.points.size());

  return result;

}

pcl::PointCloud<pcl::PointWithScale> calculate_sift_from_rgb(pcl::PointCloud<PointT>::Ptr cloud_in){

  // Parameters for sift computation
  const float min_scale = 0.01f;
  const int n_octaves = 3;
  const int n_scales_per_octave = 4;
  const float min_contrast = 0.001f;
  ROS_INFO("Comecando calculo SIFT....");
  // Estimate the sift interest points using normals values from xyz as the Intensity variants
  pcl::SIFTKeypoint<PointT, pcl::PointWithScale> sift;
  pcl::PointCloud<pcl::PointWithScale> result;
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT> ());
  sift.setSearchMethod(tree);
  sift.setScales(min_scale, n_octaves, n_scales_per_octave);
  sift.setMinimumContrast(min_contrast);
  sift.setInputCloud(cloud_in);
  sift.compute(result);

  ROS_INFO("SIFT calculado com %d pontos!", result.points.size());

  return result;

}

pcl::PointCloud<pcl::PFHSignature125>::Ptr calculate_descriptors(pcl::PointCloud<PointT>::Ptr cloud_in, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::PointWithScale>::Ptr kp, float feature_radius){

  pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors(new pcl::PointCloud<pcl::PFHSignature125>);
  pcl::PFHEstimation<PointT, pcl::Normal, pcl::PFHSignature125> pfh_est;

  pfh_est.setSearchMethod(pcl::search::KdTree<PointT>::Ptr(new pcl::search::KdTree<PointT>));
  pfh_est.setRadiusSearch(feature_radius);

  ROS_INFO("Extraindo features...");
  pcl::PointCloud<PointT>::Ptr kp_rgb(new pcl::PointCloud<PointT>);
  // Copy data to retain the keypoints
  pcl::copyPointCloud(*kp, *kp_rgb);
  // we set the surface to be searched as the whole cloud, but only search at the kp defines
  pfh_est.setSearchSurface(cloud_in);
  pfh_est.setInputNormals(normals);
  pfh_est.setInputCloud(kp_rgb);
  // Compute results
  pfh_est.compute(*descriptors);

  return descriptors;

}

pcl::CorrespondencesPtr correspondences_between_ptclouds(pcl::PointCloud<pcl::PFHSignature125>::Ptr src, pcl::PointCloud<pcl::PFHSignature125>::Ptr tgt){

  // Calculating correspondences
  pcl::registration::CorrespondenceEstimation<pcl::PFHSignature125, pcl::PFHSignature125> est;
  pcl::CorrespondencesPtr cor(new pcl::Correspondences());

  est.setInputSource(src);
  est.setInputTarget(tgt);
  est.determineCorrespondences(*cor);

  // Rejecting duplicates


}



void cloud_open_target(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  //declare variables
  pcl::PointCloud<PointT>::Ptr cloud;
  pcl::PointCloud<PointT>::Ptr cloud_transformed;
  pcl::PointCloud<PointT>::Ptr tmp_cloud;
  pcl::VoxelGrid<PointT> grid;
  sensor_msgs::PointCloud2 msg_out;

  //allocate objects for pointers
  cloud = (pcl::PointCloud<PointT>::Ptr) (new pcl::PointCloud<PointT>);
  cloud_transformed = (pcl::PointCloud<PointT>::Ptr) (new pcl::PointCloud<PointT>);
  tmp_cloud = (pcl::PointCloud<PointT>::Ptr) (new pcl::PointCloud<PointT>);

  //Convert the ros message to pcl point cloud
  pcl::fromROSMsg (*msg, *cloud);

  //Remove any NAN points in the cloud
  std::vector<int> indicesNAN;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, indicesNAN);

  // Filter for color
//  cloud = filter_color(cloud);

  //Get the transform, return if cannot get it
  ros::Time tic = ros::Time::now();
  ros::Time t = msg->header.stamp;
  tf::StampedTransform trans;
  try
  {
    p_listener->waitForTransform(ros::names::remap("/map"), msg->header.frame_id, t, ros::Duration(3.0));
    p_listener->lookupTransform(ros::names::remap("/map"), msg->header.frame_id, t, trans);
  }
  catch (tf::TransformException& ex){
    ROS_ERROR("%s",ex.what());
    //ros::Duration(1.0).sleep();
    ROS_WARN("Cannot accumulate");
    return;
  }
  ROS_INFO("Collected transforms (%0.3f secs)", (ros::Time::now() - tic).toSec());


  //Transform point cloud using the transform obtained
  Eigen::Affine3d eigen_trf;
  tf::transformTFToEigen (trans, eigen_trf);
  pcl::transformPointCloud<PointT>(*cloud, *cloud_transformed, eigen_trf);

  //Filter the transformed cloud before accumulate with voxel grid
  //    *tmp_cloud = *cloud_transformed;
  //    grid.setInputCloud(tmp_cloud);
  //    grid.setLeafSize(0.05f, 0.05f, 0.05f);
  //    grid.filter(*cloud_transformed);
  //Accumulate the point cloud using the += operator
  pcl::PointCloud<pcl::Normal>::Ptr nor(new pcl::PointCloud<pcl::Normal>);
  nor = calculate_normals(cloud_transformed);
  ROS_INFO("Size of cloud_transformed = %ld", cloud_transformed->points.size());
  (*accumulated_cloud) += (*cloud_transformed);

  //Voxel grid filter the accumulated cloud
  //    *tmp_cloud = *accumulated_cloud;
  //    grid.setInputCloud(tmp_cloud);
  //    grid.setLeafSize (0.2f, 0.2f, 0.2f);
  //    grid.setLeafSize (0.05f, 0.05f, 0.05f);
  //    grid.filter (*accumulated_cloud);
  ROS_INFO("Size of accumulated_cloud = %ld", accumulated_cloud->points.size());

  //Conver the pcl point cloud to ros msg and publish
  pcl::toROSMsg (*accumulated_cloud, msg_out);
  msg_out.header.stamp = t;
  pub->publish(msg_out);

  cloud.reset();
  tmp_cloud.reset();
  cloud_transformed.reset();
}


int main (int argc, char** argv)
{

  // Initialize ROS
  ros::init (argc, argv, "accumulatepointcloud");
  ros::NodeHandle nh;

  //initialize the transform listenerand wait a bit
  p_listener = (tf::TransformListener*) new tf::TransformListener;
  ros::Duration(2).sleep();

  //Initialize accumulated cloud variable
  accumulated_cloud = (pcl::PointCloud<PointT>::Ptr) new pcl::PointCloud<PointT>;
  accumulated_cloud->header.frame_id = ros::names::remap("/map");

  //Initialize the point cloud publisher
  pub = (boost::shared_ptr<ros::Publisher>) new ros::Publisher;
  *pub = nh.advertise<sensor_msgs::PointCloud2>("/accumulated_point_cloud", 1);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub_target = nh.subscribe ("input", 10, cloud_open_target);

  //Loop infinitly
  while (ros::ok())
  {
    // Spin
    ros::spinOnce();
  }
  //Save accumulated point cloud to a file
  printf("Saving to file %s\n", filename.c_str());
  pcl::io::savePCDFileASCII (filename, *accumulated_cloud);

  return 0;

}
