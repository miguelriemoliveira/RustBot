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

#include <pcl/keypoints/iss_3d.h>
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
#include <pcl/sample_consensus/ransac.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_svd_scale.h>
#include <pcl/registration/icp.h>

//Definitions
typedef pcl::PointXYZRGB PointT;
typedef pcl::PFHSignature125 algorithm;

//Global vars
std::string filename  = "/tmp/output.pcd";
pcl::PointCloud<PointT>::Ptr accumulated_cloud;
pcl::PointCloud<PointT>::Ptr backup_compare; // compare features from last cloud
pcl::PointCloud<PointT>::Ptr second;

tf::TransformListener *p_listener;
boost::shared_ptr<ros::Publisher> pub;

bool use_icp = true; // Here to try simple icp instead of whole feature process

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
  ne.setRadiusSearch(2);
  ne.compute(*cloud_normals);
  ROS_INFO("Retornando nuvem com normais calculadas!");

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_pointnormals (new pcl::PointCloud<pcl::PointNormal>);
  cloud_pointnormals->resize(cloud_normals->points.size());
  pcl::concatenateFields(*cloud_xyz_ptr, *cloud_normals, *cloud_pointnormals);

  return cloud_pointnormals;

}

pcl::PointCloud<pcl::Normal>::Ptr calculate_normals(pcl::PointCloud<PointT>::Ptr cloud_in){

//  ROS_INFO("Extraindo coordenadas...");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_ptr (new pcl::PointCloud<pcl::PointXYZ>); // Gather only pose so can compute normals
  cloud_xyz_ptr->resize(cloud_in->points.size()); // allocate memory space
  for(int i=0; i < cloud_in->points.size(); i++){
    cloud_xyz_ptr->points[i].x = cloud_in->points[i].x;
    cloud_xyz_ptr->points[i].y = cloud_in->points[i].y;
    cloud_xyz_ptr->points[i].z = cloud_in->points[i].z;
  }
//  ROS_INFO("Calculando normais.......");
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_n(new pcl::search::KdTree<pcl::PointXYZ>());

  ne.setInputCloud(cloud_xyz_ptr);
  ne.setSearchMethod(tree_n);
  ne.setRadiusSearch(2);
  ne.compute(*cloud_normals);
//  ROS_INFO("Retornando nuvem com normais calculadas!");

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

pcl::PointCloud<pcl::PointWithScale>::Ptr calculate_sift_from_rgb(pcl::PointCloud<PointT>::Ptr cloud_in){

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
  pcl::PointCloud<pcl::PointWithScale>::Ptr result_ptr (new pcl::PointCloud<pcl::PointWithScale>());
  *result_ptr = result;

  ROS_INFO("SIFT calculado com %d pontos!", result_ptr->points.size());

  return result_ptr;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr calculate_keypoints(pcl::PointCloud<PointT>::Ptr cloud_in){

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>());
  pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss;
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypoints(new pcl::PointCloud<pcl::PointXYZ>());

  iss.setSearchMethod(tree);
  //iss.setSalientRadius(10 * model_resolution_1);
  //iss.setNonMaxRadius(8 * model_resolution_1);
  iss.setThreshold21(0.2);
  iss.setThreshold32(0.2);
  iss.setMinNeighbors(10);
  iss.setNumberOfThreads(10);

  pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>());
  temp->resize(cloud_in->points.size());
  for(int i=0; i<temp->size(); i++){
    temp->points[i].x = cloud_in->points[i].x;
    temp->points[i].y = cloud_in->points[i].y;
    temp->points[i].z = cloud_in->points[i].z;
  }

  iss.setInputCloud(temp);
  iss.compute((*target_keypoints));

  return target_keypoints;

}

pcl::PointCloud<algorithm>::Ptr calculate_descriptors(pcl::PointCloud<PointT>::Ptr cloud_in, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::PointWithScale>::Ptr kp, float feature_radius){

  pcl::PointCloud<algorithm>::Ptr descriptors(new pcl::PointCloud<algorithm>);
  pcl::PFHEstimation<PointT, pcl::Normal, algorithm> pfh_est;

  pfh_est.setSearchMethod(pcl::search::KdTree<PointT>::Ptr(new pcl::search::KdTree<PointT>));
  pfh_est.setRadiusSearch(feature_radius);

  ROS_INFO("Extraindo features...");
  pcl::PointCloud<PointT>::Ptr kp_rgb(new pcl::PointCloud<PointT>());
  // Copy data to retain the keypoints
  pcl::copyPointCloud(*kp, *kp_rgb);
  // we set the surface to be searched as the whole cloud, but only search at the kp defines
  pfh_est.setSearchSurface(cloud_in);
  pfh_est.setInputNormals(normals);
  pfh_est.setInputCloud(kp_rgb);
  // Compute results
  pfh_est.compute(*descriptors);

  ROS_INFO("Pontos no descritor: %d", descriptors->size());

  return descriptors;

}

Eigen::Matrix4f tf_from_correspondences_between_ptclouds(pcl::PointCloud<algorithm>::Ptr src,
                                                         pcl::PointCloud<algorithm>::Ptr tgt,
                                                         pcl::PointCloud<pcl::PointWithScale>::Ptr src_kp,
                                                         pcl::PointCloud<pcl::PointWithScale>::Ptr tgt_kp)
{
  // Calculating correspondences
  pcl::registration::CorrespondenceEstimation<algorithm, algorithm> est;
  pcl::CorrespondencesPtr cor(new pcl::Correspondences());

  est.setInputSource(src);
  est.setInputTarget(tgt);
  est.determineCorrespondences(*cor);
  ROS_INFO("Obtendo melhor transformacao...");
  // Reject duplicates
  pcl::CorrespondencesPtr cor_no_dup (new pcl::Correspondences());
  pcl::registration::CorrespondenceRejectorOneToOne rej;

  rej.setInputCorrespondences(cor);
  rej.getCorrespondences(*cor_no_dup);

  // Rejecting outliers via ransac algorithm
  pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointWithScale> corr_rej_sac;
  pcl::CorrespondencesPtr cor_ransac(new pcl::Correspondences());
  corr_rej_sac.setInputSource(src_kp);
  corr_rej_sac.setInputTarget(tgt_kp);
  corr_rej_sac.setInlierThreshold(2.5);
  corr_rej_sac.setMaximumIterations(1000);
  corr_rej_sac.setRefineModel(false);
  corr_rej_sac.setInputCorrespondences(cor);
  corr_rej_sac.getCorrespondences(*cor_ransac);
  Eigen::Matrix4f transf;
  transf = corr_rej_sac.getBestTransformation();

  //  pcl::registration::TransformationEstimationSVDScale<pcl::PointWithScale, pcl::PointWithScale> te;
  //  te.estimateRigidTransformation(*src_kp, *tgt_kp, *cor_no_dup, transf);

  ROS_INFO("Melhor transformacao obtida...");
  return transf;

}

void cloud_open_target(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  //declare variables
  pcl::PointCloud<PointT>::Ptr cloud;
  pcl::PointCloud<PointT>::Ptr cloud_transformed;
  pcl::PointCloud<PointT>::Ptr cloud_adjust;
  pcl::PointCloud<PointT>::Ptr tmp_cloud;
  pcl::VoxelGrid<PointT> grid;
  sensor_msgs::PointCloud2 msg_out;
  //allocate objects for pointers
  cloud = (pcl::PointCloud<PointT>::Ptr) (new pcl::PointCloud<PointT>);
  cloud_transformed = (pcl::PointCloud<PointT>::Ptr) (new pcl::PointCloud<PointT>);
  tmp_cloud = (pcl::PointCloud<PointT>::Ptr) (new pcl::PointCloud<PointT>);
  cloud_adjust = (pcl::PointCloud<PointT>::Ptr) (new pcl::PointCloud<PointT>);

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
//  ROS_INFO("Collected transforms (%0.3f secs)", (ros::Time::now() - tic).toSec());


  //Transform point cloud using the transform obtained
  Eigen::Affine3d eigen_trf;
  tf::transformTFToEigen (trans, eigen_trf);
  pcl::transformPointCloud<PointT>(*cloud, *cloud_transformed, eigen_trf);
  Eigen::Matrix4f temp_trans = eigen_trf.cast<float>().matrix();

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// From here on we have the pipeline to accumulate from cloud registrarion using features obtained, so we have better result ///
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if(accumulated_cloud->points.size() < 3)
  { // Here we have no cloud yet, first round
    (*accumulated_cloud) += (*cloud_transformed);
    backup_compare = cloud_transformed;
  } else { // Now the actual pipeline
    if(use_icp){ // Try to use icp
      // Get normals and concatenate
      pcl::PointCloud<pcl::Normal>::Ptr src_nor(new pcl::PointCloud<pcl::Normal>());
      pcl::PointCloud<pcl::Normal>::Ptr tgt_nor(new pcl::PointCloud<pcl::Normal>());
      src_nor = calculate_normals(cloud_transformed);
      tgt_nor = calculate_normals(backup_compare);
      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr src_rgb_nor(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr tgt_rgb_nor(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
      pcl::concatenateFields(*cloud_transformed, *src_nor, *src_rgb_nor);
      pcl::concatenateFields(*backup_compare, *tgt_nor, *tgt_rgb_nor);
      pcl::removeNaNFromPointCloud(*src_rgb_nor, *src_rgb_nor, indicesNAN);
      pcl::removeNaNFromPointCloud(*tgt_rgb_nor, *tgt_rgb_nor, indicesNAN);
      pcl::removeNaNNormalsFromPointCloud(*src_rgb_nor, *src_rgb_nor, indicesNAN);
      pcl::removeNaNNormalsFromPointCloud(*tgt_rgb_nor, *tgt_rgb_nor, indicesNAN);
      // Calculate transform after alignment
      pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;
      icp.setInputSource(src_rgb_nor);
      icp.setInputTarget(tgt_rgb_nor);
      // Final result, passing eigen_trf as first guess, which seems to be actually close enough
      pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_aligned;
      icp.align(cloud_aligned, temp_trans);
      if(cloud_aligned.is_dense){
        Eigen::Matrix4f transf_icp = icp.getFinalTransformation();
        ROS_INFO("Convergimos no metodo? %d", icp.hasConverged());
        // Transform the source cloud so we add afterwards and renew backup
        pcl::transformPointCloud<PointT>(*cloud_transformed, *cloud_adjust, transf_icp);
      } else {
        cloud_adjust = cloud_transformed;
      }
      (*accumulated_cloud) += (*cloud_adjust);
      backup_compare = cloud_adjust;
    } else {
      // Normals
      pcl::PointCloud<pcl::Normal>::Ptr src_nor(new pcl::PointCloud<pcl::Normal>());
      pcl::PointCloud<pcl::Normal>::Ptr tgt_nor(new pcl::PointCloud<pcl::Normal>());
      src_nor = calculate_normals(cloud_transformed);
      tgt_nor = calculate_normals(backup_compare);
      // Keypoints
      pcl::PointCloud<pcl::PointWithScale>::Ptr src_kp_sift (new pcl::PointCloud<pcl::PointWithScale>());
      pcl::PointCloud<pcl::PointWithScale>::Ptr tgt_kp_sift (new pcl::PointCloud<pcl::PointWithScale>());
      src_kp_sift = calculate_sift_from_rgb(cloud_transformed);
      tgt_kp_sift = calculate_sift_from_rgb(backup_compare);
      // Features (descriptors)
      pcl::PointCloud<algorithm>::Ptr src_features (new pcl::PointCloud<algorithm>());
      pcl::PointCloud<algorithm>::Ptr tgt_features (new pcl::PointCloud<algorithm>());
      src_features = calculate_descriptors(cloud_transformed, src_nor, src_kp_sift, 0.05);
      tgt_features = calculate_descriptors(backup_compare, tgt_nor, tgt_kp_sift, 0.05);
      // Compare features and transform cloud
      Eigen::Matrix4f transf = tf_from_correspondences_between_ptclouds(src_features, tgt_features, src_kp_sift, tgt_kp_sift);
      // Transform cloud, accumulate it and renew backup
      pcl::transformPointCloud<PointT>(*cloud_transformed, *cloud_adjust, transf);
      (*accumulated_cloud) += (*cloud_adjust);
      backup_compare = cloud_adjust;
    }
  }

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  //Filter the transformed cloud before accumulate with voxel grid
  //    *tmp_cloud = *cloud_transformed;
  //    grid.setInputCloud(tmp_cloud);
  //    grid.setLeafSize(0.05f, 0.05f, 0.05f);
  //    grid.filter(*cloud_transformed);
  //Accumulate the point cloud using the += operator
  ROS_INFO("Size of cloud_transformed = %ld", cloud_transformed->points.size());
  ROS_INFO("Size of accumulated_cloud = %ld", accumulated_cloud->points.size());
//  (*accumulated_cloud) += (*cloud_transformed);

  //Conver the pcl point cloud to ros msg and publish
  pcl::toROSMsg (*accumulated_cloud, msg_out);
  msg_out.header.stamp = t;
  pub->publish(msg_out);

  cloud.reset();
  tmp_cloud.reset();
  cloud_transformed.reset();
  cloud_adjust.reset();
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
