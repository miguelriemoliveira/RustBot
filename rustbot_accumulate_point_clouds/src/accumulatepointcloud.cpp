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
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/registration/icp.h>

#include <algorithm>

//Definitions
typedef pcl::PointXYZRGB PointT;

//Global vars
std::string filename  = "/tmp/output.pcd";

pcl::PointCloud<PointT>::Ptr accumulated_cloud;
pcl::PointCloud<PointT>::Ptr first; // Aqui para tentar acumular arredondando pontos semelhantes ou proximos
pcl::PointCloud<PointT>::Ptr second;

tf::TransformListener *p_listener;
boost::shared_ptr<ros::Publisher> pub;

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

  // Try to clear white and blue points from sky, and green ones from the grass, or something close to it
  int rMax = 200;
  int rMin = 0;
  int gMax = 100;
  int gMin = 0;
  int bMax = 100;
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
  condrem.setInputCloud (cloud);
  condrem.setKeepOrganized(true);

  // apply filter
  condrem.filter (*cloud);

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
  *tmp_cloud = *cloud_transformed;
  grid.setInputCloud(tmp_cloud);
  grid.setLeafSize(0.2f, 0.2f, 0.2f);
  grid.filter(*cloud_transformed);
  //Remove outliers
  pcl::RadiusOutlierRemoval<PointT> ror;
  ror.setInputCloud(cloud_transformed);
  ror.setRadiusSearch(15);
  ror.setMinNeighborsInRadius(5);
  ror.filter(*cloud_transformed);
  //    ROS_INFO("Size of cloud_transformed = %ld", cloud_transformed->points.size());

  *second = *first;
  *first  = *cloud_transformed;
  first->points.resize(std::min(first->width, second->width) * std::min(first->height, second->height));
  second->points.resize(first->width * second->height);

  // Approximate two subsequent clouds by the average of points position
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setInputSource(first);
  icp.setInputTarget(second);

  pcl::PointCloud<PointT> avg;
  icp.align(avg);
  pcl::PointCloud<PointT>::Ptr avg_ptr;
  avg_ptr = (pcl::PointCloud<PointT>::Ptr) (new pcl::PointCloud<PointT>);
  *avg_ptr = avg;

  //Accumulate the point cloud using the += operator
  ROS_INFO("Size of cloud_transformed = %ld", avg_ptr->points.size());

  (*accumulated_cloud) += (*avg_ptr);

  //    (*accumulated_cloud) += (*cloud_transformed);

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

  //initialize the transform listener and wait a bit
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

  // Iniciando somente as nuvens de fusao
  first  = (pcl::PointCloud<PointT>::Ptr) (new pcl::PointCloud<PointT>);
  second = (pcl::PointCloud<PointT>::Ptr) (new pcl::PointCloud<PointT>);

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
