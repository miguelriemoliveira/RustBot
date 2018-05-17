//Includes
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/keypoints/harris_3d.h>

#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_estimation_backprojection.h>
#include <pcl/registration/correspondence_rejection.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_distance.h>
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
#include <pcl/registration/icp_nl.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/common/angles.h>
#include <math.h>

using namespace std;
using namespace pcl;
using namespace pcl::registration;

//Definitions
typedef pcl::PointXYZRGB PointT;


//Global vars
pcl::PointCloud<PointT>::Ptr accumulated_cloud (new pcl::PointCloud<PointT>());
pcl::PointCloud<PointT>::Ptr backup_compare (new pcl::PointCloud<PointT>()); // compare features from last cloud

boost::shared_ptr<ros::Publisher> pub;

boost::shared_ptr<pcl::visualization::PCLVisualizer> vis_all (new pcl::visualization::PCLVisualizer ("all correspondences"));
boost::shared_ptr<pcl::visualization::PCLVisualizer> vis_rej (new pcl::visualization::PCLVisualizer ("filter correspondences"));
boost::shared_ptr<pcl::visualization::PCLVisualizer> vis_fim (new pcl::visualization::PCLVisualizer ("matched"));

float lf = 0.2f; // Leaf size for voxel grid

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void filter_color(pcl::PointCloud<PointT>::Ptr cloud_in){

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
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void simplify_cloud_xyz(pcl::PointCloud<PointT>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out){
  cloud_out->resize(cloud_in->size());
  for(int i=0; i<cloud_in->points.size(); i++){
    cloud_out->points[i].x = cloud_in->points[i].x;
    cloud_out->points[i].y = cloud_in->points[i].y;
    cloud_out->points[i].z = cloud_in->points[i].z;
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void passthrough(pcl::PointCloud<PointT>::Ptr in, std::string field, float min, float max){
  pcl::PassThrough<PointT> ps;
  ps.setInputCloud(in);
  ps.setFilterFieldName(field);
  ps.setFilterLimits(min, max);

  ps.filter(*in);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void remove_outlier(pcl::PointCloud<PointT>::Ptr in, float mean, float deviation){
  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud(in);
  sor.setMeanK(mean);
  sor.setStddevMulThresh(deviation);
//  pcl::PointCloud<PointT>::Ptr temp (new pcl::PointCloud<PointT>() );
  sor.filter(*in);

//  pcl::RadiusOutlierRemoval<PointT> outrem;
//  outrem.setInputCloud(temp);
//  outrem.setRadiusSearch(1);
//  outrem.setMinNeighborsInRadius(5);
//  outrem.filter(*in);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calculate_pointnormals(pcl::PointCloud<PointT>::Ptr cloud_in, int k, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_pointnormals){

  ROS_INFO("Extraindo coordenadas...");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_ptr (new pcl::PointCloud<pcl::PointXYZ>() ); // Gather only pose so can compute normals
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
  ne.setKSearch(k);
  ne.compute(*cloud_normals);

  pcl::concatenateFields(*cloud_xyz_ptr, *cloud_normals, *cloud_pointnormals);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calculate_normals(pcl::PointCloud<PointT>::Ptr cloud_in, int k, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals){

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_ptr (new pcl::PointCloud<pcl::PointXYZ>()); // Gather only pose so can compute normals
  cloud_xyz_ptr->resize(cloud_in->points.size()); // allocate memory space
  ROS_INFO("size: %d", cloud_in->points.size());
  for(int i=0; i < cloud_in->points.size(); i++){
    cloud_xyz_ptr->points[i].x = cloud_in->points[i].x;
    cloud_xyz_ptr->points[i].y = cloud_in->points[i].y;
    cloud_xyz_ptr->points[i].z = cloud_in->points[i].z;
  }
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_n(new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setInputCloud(cloud_xyz_ptr);
  ne.setSearchMethod(tree_n);
  ne.setKSearch(k);
  ne.compute(*cloud_normals);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calculate_xyzrgbnormals(pcl::PointCloud<PointT>::Ptr cloud_in, int k, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_xyzrgbnormals){

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_ptr (new pcl::PointCloud<pcl::PointXYZ>()); // Gather only pose so can compute normals
  cloud_xyz_ptr->resize(cloud_in->points.size()); // allocate memory space
  for(int i=0; i < cloud_in->points.size(); i++){
    cloud_xyz_ptr->points[i].x = cloud_in->points[i].x;
    cloud_xyz_ptr->points[i].y = cloud_in->points[i].y;
    cloud_xyz_ptr->points[i].z = cloud_in->points[i].z;
  }
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_n(new pcl::search::KdTree<pcl::PointXYZ>());
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>());
  cloud_normals->resize(cloud_in->points.size()); // allocate memory space

  ne.setInputCloud(cloud_xyz_ptr);
  ne.setSearchMethod(tree_n);
  ne.setKSearch(k);
  ne.compute(*cloud_normals);

  pcl::concatenateFields(*cloud_in, *cloud_normals, *cloud_xyzrgbnormals);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void findCorrespondences2 (const pcl::PointCloud<PointT>::Ptr src,
                          const pcl::PointCloud<PointT>::Ptr tgt,
                          const pcl::PointCloud<pcl::Normal>::Ptr src_normal,
                          const pcl::PointCloud<pcl::Normal>::Ptr tgt_normal,
                          pcl::CorrespondencesPtr all_correspondences)
{
  //CorrespondenceEstimationNormalShooting<PointT, PointT, PointT> est;
  //CorrespondenceEstimation<PointT, PointT> est;
  pcl::registration::CorrespondenceEstimationBackProjection<PointT, PointT, pcl::Normal> est;
  est.setInputSource(src);
  est.setInputTarget(tgt);

  est.setSourceNormals(src_normal);
  est.setTargetNormals(tgt_normal);
  est.setKSearch(10);
  est.determineCorrespondences(*all_correspondences);
  //est.determineReciprocalCorrespondences (all_correspondences);
  ROS_INFO("correspondencias: [%d]", all_correspondences->size());

//  pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_src(src);
//  pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_tgt(tgt);
//  vis_all->removePointCloud("src");
//  vis_all->addPointCloud<PointT>(src, rgb_src, "src");
//  vis_all->removePointCloud("tgt");
//  vis_all->addPointCloud<PointT>(tgt, rgb_tgt, "tgt");
//  if(!vis_all->updatePointCloud<PointT>(src, "src")) vis_all->addPointCloud<PointT>(src, rgb_src, "src");
//  vis_all->resetCameraViewpoint("src");
//  if(!vis_all->updatePointCloud<PointT>(tgt, "tgt")) vis_all->addPointCloud<PointT>(tgt, rgb_tgt, "tgt");
//  if(!vis_all->updateCorrespondences<PointT>(src, tgt, *all_correspondences, 0))
//    vis_all->addCorrespondences<PointT>(src, tgt, *all_correspondences, 0, "correspondences");
//  vis_all->spinOnce();
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void rejectBadCorrespondences2 (const pcl::CorrespondencesPtr all_correspondences,
                               const pcl::PointCloud<PointT>::Ptr src,
                               const pcl::PointCloud<PointT>::Ptr tgt,
                               const pcl::PointCloud<pcl::Normal>::Ptr src_normal,
                               const pcl::PointCloud<pcl::Normal>::Ptr tgt_normal,
                               pcl::CorrespondencesPtr remaining_correspondences,
                               Eigen::Matrix4f &transf)
{
  pcl::registration::CorrespondenceRejectorMedianDistance rej;
  rej.setMedianFactor (0.1);
  rej.setInputCorrespondences (all_correspondences);
  pcl::CorrespondencesPtr remaining_correspondences_temp (new pcl::Correspondences);
  rej.getCorrespondences (*remaining_correspondences_temp);
  ROS_INFO("Number of correspondences remaining after rejection median: %d\n", remaining_correspondences_temp->size ());

  // Reject if the angle between the normals is really off
  pcl::registration::CorrespondenceRejectorSurfaceNormal rej_normals;
  rej_normals.setThreshold(pcl::deg2rad(15.0f));
  rej_normals.initializeDataContainer<PointT, pcl::Normal> ();
  rej_normals.setInputSource<PointT> (src);
  rej_normals.setInputNormals<PointT, pcl::Normal> (src_normal);
  rej_normals.setInputTarget<PointT> (tgt);
  rej_normals.setTargetNormals<PointT, pcl::Normal> (tgt_normal);
  rej_normals.setInputCorrespondences (remaining_correspondences_temp);/*
  pcl::CorrespondencesPtr remaining_correspondences_temp2 (new pcl::Correspondences);
  rej.getCorrespondences(*remaining_correspondences_temp2); // So it doesnt crash on execution*/
  rej_normals.getCorrespondences (*remaining_correspondences_temp);
  ROS_INFO("Number of correspondences remaining after rejection normals: %d\n", remaining_correspondences_temp->size ());

  pcl::registration::CorrespondenceRejectorOneToOne rej_one;
  rej_one.setInputCorrespondences(remaining_correspondences_temp);
//  pcl::CorrespondencesPtr remaining_correspondences_temp3 (new pcl::Correspondences() );
  rej_one.getRemainingCorrespondences(*remaining_correspondences_temp, *remaining_correspondences_temp);
  ROS_INFO("Number of correspondences remaining after rejection one to one: %d\n", remaining_correspondences_temp->size ());

  pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> corr_rej_sac;
  corr_rej_sac.setInputSource(src);
  corr_rej_sac.setInputTarget(tgt);
  corr_rej_sac.setInlierThreshold(0.00005);
  corr_rej_sac.setMaximumIterations(100);
  corr_rej_sac.setRefineModel(false);
  corr_rej_sac.setInputCorrespondences(remaining_correspondences_temp);
  corr_rej_sac.getCorrespondences(*remaining_correspondences);
  ROS_INFO("Number of correspondences remaining after rejection ransac: %d\n", remaining_correspondences->size ());

  transf = corr_rej_sac.getBestTransformation();
  ROS_INFO("DENTRO DA MATRIZ: %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f", transf(0, 0), transf(0, 1), transf(0, 2), transf(0, 3), transf(1, 0), transf(1, 1), transf(1, 2), transf(1, 3), transf(2, 0), transf(2, 1), transf(2, 2), transf(2, 3));
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void findTransformation2 (const pcl::PointCloud<PointT>::Ptr src,
                         const pcl::PointCloud<PointT>::Ptr tgt,
                         pcl::PointCloud<pcl::Normal>::Ptr src_normal,
                         pcl::PointCloud<pcl::Normal>::Ptr tgt_normal,
                         pcl::CorrespondencesPtr correspondences,
                         Eigen::Matrix4f &transf)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr src_xyz (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_xyz (new pcl::PointCloud<pcl::PointXYZ> ());
  simplify_cloud_xyz(src, src_xyz);
  simplify_cloud_xyz(tgt, tgt_xyz);
  pcl::PointCloud<pcl::PointNormal>::Ptr src_xyzn (new pcl::PointCloud<pcl::PointNormal> ());
  pcl::PointCloud<pcl::PointNormal>::Ptr tgt_xyzn (new pcl::PointCloud<pcl::PointNormal> ());
  src_xyzn->resize(src->points.size());
  tgt_xyzn->resize(tgt->points.size());
  pcl::concatenateFields(*src_xyz, *src_normal, *src_xyzn);
  pcl::concatenateFields(*tgt_xyz, *tgt_normal, *tgt_xyzn);

  pcl::registration::TransformationEstimationSVD<pcl::PointNormal, pcl::PointNormal> trans_est;
  trans_est.estimateRigidTransformation(*src_xyzn, *tgt_xyzn, *correspondences, transf);//trans_est.estimateRigidTransformation();
  ROS_INFO("DENTRO DA MATRIZ 2: %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f", transf(0, 0), transf(0, 1), transf(0, 2), transf(0, 3), transf(1, 0), transf(1, 1), transf(1, 2), transf(1, 3), transf(2, 0), transf(2, 1), transf(2, 2), transf(2, 3));
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void estimateKeypoints (const PointCloud<PointT>::Ptr src,
                        const PointCloud<PointT>::Ptr tgt,
                        const PointCloud<Normal>::Ptr normals_src,
                        const PointCloud<Normal>::Ptr normals_tgt,
                        PointCloud<PointXYZI>::Ptr keypoints_src,
                        PointCloud<PointXYZI>::Ptr keypoints_tgt,
                        const PointCloud<Normal>::Ptr normals_src_filt,
                        const PointCloud<Normal>::Ptr normals_tgt_filt,
                        PointIndicesConstPtr src_ind,
                        PointIndicesConstPtr tgt_ind,
                        int k)
{
  HarrisKeypoint3D<PointXYZRGB, PointXYZI> harris;
  harris.setRadius(1);
  harris.setNumberOfThreads(8);
  harris.setThreshold(1e-6); // Test

  harris.setInputCloud(src);
  harris.compute(*keypoints_src);
  src_ind = harris.getKeypointsIndices();
  harris.setInputCloud(tgt);
  harris.compute(*keypoints_tgt);
  tgt_ind = harris.getKeypointsIndices();

  ROS_INFO("Quantos keypoints: %d", keypoints_tgt->size());
  ROS_INFO("Quantos indices: %d", tgt_ind->indices.size());

  ExtractIndices<Normal> extract;
  extract.setNegative(false);
  extract.setInputCloud(normals_src);
  extract.setIndices(src_ind);
  extract.filter(*normals_src_filt);
  extract.setInputCloud(normals_tgt);
  extract.setIndices(tgt_ind);
  extract.filter(*normals_tgt_filt);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void estimateFPFH (const PointCloud<PointT>::Ptr src,
                   const PointCloud<PointT>::Ptr tgt,
                   const PointCloud<Normal>::Ptr normals_src,
                   const PointCloud<Normal>::Ptr normals_tgt,
                   const PointCloud<PointXYZI>::Ptr keypoints_src,
                   const PointCloud<PointXYZI>::Ptr keypoints_tgt,
                   PointCloud<FPFHSignature33>::Ptr fpfhs_src,
                   PointCloud<FPFHSignature33>::Ptr fpfhs_tgt)
{
  ROS_INFO("Quantos indices normal: %d", normals_tgt->size());
  FPFHEstimation<PointXYZI, Normal, FPFHSignature33> fpfh_est;
  pcl::search::KdTree<PointXYZI>::Ptr tree (new pcl::search::KdTree<PointXYZI>);
  fpfh_est.setSearchMethod(tree);
  fpfh_est.setInputCloud(keypoints_src);
  fpfh_est.setInputNormals(normals_src);
//  fpfh_est.setRadiusSearch(1); // 1m
  fpfh_est.setKSearch(20);
  PointCloud<PointXYZI>::Ptr src_xyzi (new PointCloud<PointXYZI> ());
  PointCloudXYZRGBtoXYZI(*src, *src_xyzi);
  fpfh_est.setSearchSurface(src_xyzi);
  fpfh_est.compute(*fpfhs_src);

  fpfh_est.setInputCloud(keypoints_tgt);
  fpfh_est.setInputNormals(normals_tgt);
  PointCloud<PointXYZI>::Ptr tgt_xyzi (new PointCloud<PointXYZI> ());
  PointCloudXYZRGBtoXYZI(*tgt, *tgt_xyzi);
  fpfh_est.setSearchSurface(tgt_xyzi);
  fpfh_est.compute(*fpfhs_tgt);

  ROS_INFO("Tamanho das features: %d", fpfhs_tgt->size());
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void findCorrespondences (const PointCloud<FPFHSignature33>::Ptr fpfhs_src,
                          const PointCloud<FPFHSignature33>::Ptr fpfhs_tgt,
                          CorrespondencesPtr all_correspondences)
{
  CorrespondenceEstimation<FPFHSignature33, FPFHSignature33> est;
  est.setInputSource(fpfhs_src);
  est.setInputTarget(fpfhs_tgt);
  est.determineReciprocalCorrespondences(*all_correspondences);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void rejectBadCorrespondences (const CorrespondencesPtr all_correspondences,
                               const PointCloud<PointXYZI>::Ptr keypoints_src,
                               const PointCloud<PointXYZI>::Ptr keypoints_tgt,
                               CorrespondencesPtr remaining_correspondences)
{
  CorrespondenceRejectorDistance rej;
  rej.setInputSource<PointXYZI>(keypoints_src);
  rej.setInputTarget<PointXYZI>(keypoints_tgt);
  rej.setMaximumDistance(0.5);
  rej.setInputCorrespondences(all_correspondences);
  rej.getCorrespondences(*remaining_correspondences);
  ROS_INFO("Number of correspondences remaining after rejection distance: %d\n", remaining_correspondences->size ());

  pcl::registration::CorrespondenceRejectorSampleConsensus<PointXYZI> corr_rej_sac;
  corr_rej_sac.setInputSource(keypoints_src);
  corr_rej_sac.setInputTarget(keypoints_tgt);
  corr_rej_sac.setInlierThreshold(0.00005);
  corr_rej_sac.setMaximumIterations(100);
  corr_rej_sac.setRefineModel(false);
  corr_rej_sac.setInputCorrespondences(remaining_correspondences);
  corr_rej_sac.getCorrespondences(*remaining_correspondences);
  ROS_INFO("Number of correspondences remaining after rejection ransac: %d\n", remaining_correspondences->size ());
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void computeTransformation (const PointCloud<PointXYZI>::Ptr keypoints_src,
                            const PointCloud<PointXYZI>::Ptr keypoints_tgt,
                            const CorrespondencesPtr good_correspondences,
                            Eigen::Matrix4f &transf)
{
  TransformationEstimationSVD<PointXYZI, PointXYZI> trans_est;
  trans_est.estimateRigidTransformation (*keypoints_src, *keypoints_tgt, *good_correspondences, transf);
  ROS_INFO("DENTRO DA MATRIZ 2: %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f", transf(0, 0), transf(0, 1), transf(0, 2), transf(0, 3), transf(1, 0), transf(1, 1), transf(1, 2), transf(1, 3), transf(2, 0), transf(2, 1), transf(2, 2), transf(2, 3));

}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Here we have the function where we accumulate using the best technique to our knowledge
///
void cloud_accumulate(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  //declare variables
  PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>());
  PointCloud<PointT>::Ptr cloud_transformed (new pcl::PointCloud<PointT>());
  PointCloud<PointT>::Ptr cloud_adjust (new pcl::PointCloud<PointT>());
  PointCloud<PointT>::Ptr tmp_cloud (new pcl::PointCloud<PointT>());
  VoxelGrid<PointT> grid;
  sensor_msgs::PointCloud2 msg_out;

  //Convert the ros message to pcl point cloud
  pcl::fromROSMsg (*msg, *cloud);

  //Remove any NAN points in the cloud
  std::vector<int> indicesNAN;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, indicesNAN);

  // SImplify here to voxel because its too big
  grid.setInputCloud(cloud);
  grid.setLeafSize(lf, lf, lf);
  grid.filter(*cloud);

  // Filter for outliers
  remove_outlier(cloud, 20, 0.1);

  // Filter for color
  //  cloud = filter_color(cloud);

  // Filter for region - PassThrough
  passthrough(cloud, "z",  0, 20);
  passthrough(cloud, "x", -3,  3);
  passthrough(cloud, "y", -4,  4);

  ros::Time t = msg->header.stamp;

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// From here on we have the pipeline to accumulate from cloud registrarion using features obtained, so we have better result ///
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if(accumulated_cloud->points.size() < 3)
  { // Here we have no cloud yet, first round
    *backup_compare = *cloud;
    *tmp_cloud = *cloud;
//    grid.setInputCloud(tmp_cloud);
//    grid.setLeafSize(lf, lf, lf);
//    grid.filter(*tmp_cloud);
    (*accumulated_cloud) = (*tmp_cloud);
  } else { // Now the actual pipeline
    // Calculate all possible clouds with normals
    pcl::PointCloud<pcl::Normal>::Ptr src_normal (new pcl::PointCloud<pcl::Normal>());
    pcl::PointCloud<pcl::Normal>::Ptr tgt_normal (new pcl::PointCloud<pcl::Normal>());
    calculate_normals(cloud, 5, src_normal);
    calculate_normals(backup_compare, 5, tgt_normal);
    // Calculate keypoints using the harris algorihtm
    PointCloud<PointXYZI>::Ptr src_kp (new PointCloud<PointXYZI> ());
    PointCloud<PointXYZI>::Ptr tgt_kp (new PointCloud<PointXYZI> ());
    PointIndicesConstPtr src_ind (new PointIndices ());
    PointIndicesConstPtr tgt_ind (new PointIndices ());
    pcl::PointCloud<pcl::Normal>::Ptr src_normal_filt (new pcl::PointCloud<pcl::Normal>());
    pcl::PointCloud<pcl::Normal>::Ptr tgt_normal_filt (new pcl::PointCloud<pcl::Normal>());
    estimateKeypoints(cloud, backup_compare, src_normal, tgt_normal, src_kp, tgt_kp, src_normal_filt, tgt_normal_filt, src_ind, tgt_ind, 10);
    // Calculate features with FPFH algorithm
    PointCloud<FPFHSignature33>::Ptr src_fpfh (new PointCloud<FPFHSignature33> ());
    PointCloud<FPFHSignature33>::Ptr tgt_fpfh (new PointCloud<FPFHSignature33> ());
    estimateFPFH(cloud, backup_compare, src_normal, tgt_normal, src_kp, tgt_kp, src_fpfh, tgt_fpfh);
    // Match the features
    CorrespondencesPtr all_correspondences (new Correspondences ());
    findCorrespondences(src_fpfh, tgt_fpfh, all_correspondences);
    // Reject bad ones
    CorrespondencesPtr good_correspondences (new Correspondences ());
    rejectBadCorrespondences(all_correspondences, src_kp, tgt_kp, good_correspondences);
    // Compute the transformation from the resultant correspondences
    Eigen::Matrix4f transformation;
    computeTransformation(src_kp, tgt_kp, good_correspondences, transformation);













//    // Determine correspondences and visualize
//    pcl::CorrespondencesPtr correspondences_raw (new pcl::Correspondences());
//    findCorrespondences(cloud, backup_compare, src_normal, tgt_normal, correspondences_raw);
//    // Reject correspondences, get first transform and visualize
//    pcl::CorrespondencesPtr correspondences_filt (new pcl::Correspondences());
//    Eigen::Matrix4f transformation_ransac;
//    rejectBadCorrespondences(correspondences_raw, cloud, backup_compare, src_normal, tgt_normal, correspondences_filt, transformation_ransac);
//    ROS_INFO("First number in matrix from ransac: %.2f", transformation_ransac(1, 1));
//    // Get best transformation from the correspondences
//    Eigen::Matrix4f transformation_fim;
//    findTransformation(cloud, backup_compare, src_normal, tgt_normal, correspondences_filt, transformation_fim);
    // Find the best transformation iteratively, visualize each iteration
    // Pass the entities to the next iteration
    *backup_compare = *cloud;
    *tmp_cloud = *cloud;
    grid.setInputCloud(tmp_cloud);
    grid.setLeafSize(lf, lf, lf);
    grid.filter(*tmp_cloud);
    (*accumulated_cloud) += (*tmp_cloud);
  }

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// End of pipeline, send the result to ROS message                                                                           ///
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // Convert the pcl point cloud to ros msg and publish
  pcl::toROSMsg (*accumulated_cloud, msg_out);
  msg_out.header.stamp = t;
  pub->publish(msg_out);

  cloud.reset();
  tmp_cloud.reset();
}

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init (argc, argv, "accumulatepointcloud");
  ros::NodeHandle nh;

  //Initialize accumulated cloud variable
  accumulated_cloud = (pcl::PointCloud<PointT>::Ptr) new pcl::PointCloud<PointT>();
  accumulated_cloud->header.frame_id = ros::names::remap("/map");

  //Initialize the point cloud publisher
  pub = (boost::shared_ptr<ros::Publisher>) new ros::Publisher;
  *pub = nh.advertise<sensor_msgs::PointCloud2>("/accumulated_point_cloud", 100);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub_target = nh.subscribe ("input", 500, cloud_accumulate);
  ROS_INFO("1");
  // Adjust visualizers
  vis_all->setBackgroundColor(0, 0, 0);
  vis_rej->setBackgroundColor(0, 0, 0);
  vis_fim->setBackgroundColor(0, 0, 0);

  vis_all->addCoordinateSystem (1.0);
  vis_rej->addCoordinateSystem (1.0);
  vis_fim->addCoordinateSystem (1.0);

  vis_all->initCameraParameters ();
  vis_rej->initCameraParameters ();
  vis_fim->initCameraParameters ();

  //Loop infinitly
  while (ros::ok())
  {
    // Spin
    ros::spinOnce();
  }

  return 0;

}
