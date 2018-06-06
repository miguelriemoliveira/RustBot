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
  //pcl::removeNaNFromPointCloud(*accumulated_cloud, *accumulated_cloud, indicesNAN);

  // Try to clear white and blue points from sky, and green ones from the grass, or something close to it
  int rMax = 200;
  int rMin = 0;
  int gMax = 120;
  int gMin = 0;
  int bMax = 120;
  int bMin = 0;

//  pcl::ConditionAnd<PointT>::Ptr color_cond (new pcl::ConditionAnd<PointT> ());
//  color_cond->addComparison (pcl::PackedRGBComparison<PointT>::Ptr (new pcl::PackedRGBComparison<PointT> ("r", pcl::ComparisonOps::LT, rMax)));
//  color_cond->addComparison (pcl::PackedRGBComparison<PointT>::Ptr (new pcl::PackedRGBComparison<PointT> ("r", pcl::ComparisonOps::GT, rMin)));
//  color_cond->addComparison (pcl::PackedRGBComparison<PointT>::Ptr (new pcl::PackedRGBComparison<PointT> ("g", pcl::ComparisonOps::LT, gMax)));
//  color_cond->addComparison (pcl::PackedRGBComparison<PointT>::Ptr (new pcl::PackedRGBComparison<PointT> ("g", pcl::ComparisonOps::GT, gMin)));
//  color_cond->addComparison (pcl::PackedRGBComparison<PointT>::Ptr (new pcl::PackedRGBComparison<PointT> ("b", pcl::ComparisonOps::LT, bMax)));
//  color_cond->addComparison (pcl::PackedRGBComparison<PointT>::Ptr (new pcl::PackedRGBComparison<PointT> ("b", pcl::ComparisonOps::GT, bMin)));

//  // build the filter
//  pcl::ConditionalRemoval<PointT> condrem (color_cond);
//  condrem.setInputCloud (cloud);
//  condrem.setKeepOrganized(true);

//  // apply filter
//  condrem.filter (*cloud);

//  for(int i=0; i<cloud->points.size(); i++){
//    //      ROS_INFO("Escala das cores: r %.2f g %.2f b %.2f", cloud->points[i].r, cloud->points[i].g, cloud->points[i].b);
//    //      if(cloud->points[i].r > 20 & cloud->points[i].g > 20 & cloud->points[i].b > 20){
//    //        ROS_INFO("Achamos um ponto");
//    cloud->erase(cloud->begin()+i);
//    cloud->points.erase(cloud->begin()+i);
//    //      }
//  }

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
