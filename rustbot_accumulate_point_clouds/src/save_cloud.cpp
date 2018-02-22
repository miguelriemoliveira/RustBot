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
#include <pcl/features/normal_3d.h> // add normals to render in MART.exe

// FIle to save
std::string filename = "/tmp/output.ply";
// Define to simplify matters
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZRGBNormal Out;

void savecloud_plus_normal_ply(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  ROS_INFO("Receiving data to save......");
  // Declare the pointer to the received cloud
  pcl::PointCloud<PointT>::Ptr recv_cloud_ptr(new pcl::PointCloud<PointT>); // Allocate memory

  // Pass the message to a pcl entity
  pcl::fromROSMsg(*msg, *recv_cloud_ptr);

  // CLouds to pass info around
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_ptr (new pcl::PointCloud<pcl::PointXYZ>); // Gather only pose so can compute normals
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne; // Entity to compute normals in the future
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ()); // Kd Tree used to divide space and compute normals
  ne.setSearchMethod (tree);
  ne.setRadiusSearch (1);
  pcl::PointCloud<pcl::Normal>::Ptr normals_cloud_ptr (new pcl::PointCloud<pcl::Normal> ()); // Pointer to the cloud of normals

  // Output cloud to save
  pcl::PointCloud<Out>::Ptr output_cloud_ptr (new pcl::PointCloud<Out> ());

  ROS_INFO("Computing normals......");
  // Grab only coordinates from income cloud
  cloud_xyz_ptr->resize(recv_cloud_ptr->points.size()); // allocate memory space
  for(int i=0; i < recv_cloud_ptr->points.size(); i++){
    cloud_xyz_ptr->points[i].x = recv_cloud_ptr->points[i].x;
    cloud_xyz_ptr->points[i].y = recv_cloud_ptr->points[i].y;
    cloud_xyz_ptr->points[i].z = recv_cloud_ptr->points[i].z;
  }
  // Compute normals from coordinates data
  ne.setInputCloud(cloud_xyz_ptr);
  normals_cloud_ptr->resize(cloud_xyz_ptr->points.size()); // allocate memory
  ne.compute(*normals_cloud_ptr);

  // Concatenate both data from income cloud plus normals and save a ply file
  output_cloud_ptr->resize(recv_cloud_ptr->points.size()); // allocate memory
  ROS_INFO("Concatenate the fields......");
  pcl::concatenateFields(*recv_cloud_ptr, *normals_cloud_ptr, *output_cloud_ptr);

  ROS_INFO("Now saving to file output.ply......");
  pcl::io::savePLYFileASCII(filename, *output_cloud_ptr);
  ROS_INFO("All safe and sound !!");

  // Kill the node after saving the ptcloud
  ros::shutdown();

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "save_cloud");
  ros::NodeHandle nh;

  ROS_INFO("We will save the data! DOnt worry...");

  ros::Subscriber sub = nh.subscribe("/accumulated_point_cloud", 1000, savecloud_plus_normal_ply);

  ros::spin();

  return 0;
}
