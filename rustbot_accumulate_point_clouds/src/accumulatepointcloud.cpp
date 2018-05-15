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
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

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
#include <pcl/registration/icp_nl.h>

//Definitions
typedef pcl::PointXYZRGB PointT;

//Global vars
pcl::PointCloud<PointT>::Ptr accumulated_cloud (new pcl::PointCloud<PointT>());
pcl::PointCloud<PointT>::Ptr backup_compare (new pcl::PointCloud<PointT>()); // compare features from last cloud

boost::shared_ptr<ros::Publisher> pub;

bool use_icp = false; // Here to try simple icp instead of whole feature process
float lf = 0.05f; // Leaf size for voxel grid

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
  sor.filter(*in);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Here we have the function where we accumulate using the best technique to our knowledge
///
void cloud_accumulate(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  //declare variables
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr cloud_transformed (new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr cloud_adjust (new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr tmp_cloud (new pcl::PointCloud<PointT>());
  pcl::VoxelGrid<PointT> grid;
  sensor_msgs::PointCloud2 msg_out;

  //Convert the ros message to pcl point cloud
  pcl::fromROSMsg (*msg, *cloud);

  //Remove any NAN points in the cloud
  std::vector<int> indicesNAN;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, indicesNAN);

  // Filter for outliers
  remove_outlier(cloud, 30, 0.1);

  // Filter for color
//  cloud = filter_color(cloud);

  // Filter for region - PassThrough
//  passthrough(cloud, "z",  0, 30);
//  passthrough(cloud, "x", -3,  3);
//  passthrough(cloud, "y", -4,  4);

  ros::Time t = msg->header.stamp;

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// From here on we have the pipeline to accumulate from cloud registrarion using features obtained, so we have better result ///
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if(accumulated_cloud->points.size() < 3)
  { // Here we have no cloud yet, first round
    *backup_compare = *cloud;
    *tmp_cloud = *cloud;
    grid.setInputCloud(tmp_cloud);
    grid.setLeafSize(0.2f, 0.2f, 0.2f);
    grid.filter(*tmp_cloud);
    (*accumulated_cloud) = (*tmp_cloud);
  } else { // Now the actual pipeline
     ROS_INFO("Implement");
  }

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// End of pipeline, send the result to ROS message                                                                           ///
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  //Conver the pcl point cloud to ros msg and publish
  pcl::toROSMsg (*accumulated_cloud, msg_out);
  msg_out.header.stamp = t;
  pub->publish(msg_out);

  cloud.reset();
  tmp_cloud.reset();
  cloud_transformed.reset();
  cloud_adjust.reset();
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

    //Loop infinitly
    while (ros::ok())
    {
      // Spin
      ros::spinOnce();
    }

    return 0;

}
