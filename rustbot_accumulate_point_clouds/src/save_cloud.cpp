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
std::string filename;
// Define to simplify matters
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZRGBNormal Out;

bool no_normals = true;

void savecloud_plus_normal_ply(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  ROS_INFO("Recebendo dados para salvar......");
  // Declare the pointer to the received cloud
  pcl::PointCloud<PointT>::Ptr recv_cloud_ptr(new pcl::PointCloud<PointT>); // Allocate memory

  // Pass the message to a pcl entity
  pcl::fromROSMsg(*msg, *recv_cloud_ptr);

  ROS_INFO("Salvando o arquivo .ply na area de trabalho......");
  // Ver o tempo para diferenciar bags gravadas automaticamente
  time_t t = time(0);
  struct tm * now = localtime( & t );
  std::string year, month, day, hour, minutes;
  year    = boost::lexical_cast<std::string>(now->tm_year + 1900);
  month   = boost::lexical_cast<std::string>(now->tm_mon );
  day     = boost::lexical_cast<std::string>(now->tm_mday);
  hour    = boost::lexical_cast<std::string>(now->tm_hour);
  minutes = boost::lexical_cast<std::string>(now->tm_min );
  std::string date = "_" + year + "_" + month + "_" + day + "_" + hour + "h_" + minutes + "m";
  filename = "/home/mrs/Desktop/pos_processo_em"+date+".ply";

  if (no_normals){ // No normals, goes faster

    pcl::PointCloud<PointT>::Ptr output_cloud_ptr (new pcl::PointCloud<PointT> ());
    output_cloud_ptr = recv_cloud_ptr;

    pcl::io::savePLYFileASCII(filename, *output_cloud_ptr);
    ROS_INFO("Tudo correto, conferir pelo arquivo na area de trabalho !!");

  } else { // Normals, able to mesh later

    // CLouds to pass info around
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_ptr (new pcl::PointCloud<pcl::PointXYZ>()); // Gather only pose so can compute normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne; // Entity to compute normals in the future
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ()); // Kd Tree used to divide space and compute normals
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (10);
    pcl::PointCloud<pcl::Normal>::Ptr normals_cloud_ptr (new pcl::PointCloud<pcl::Normal> ()); // Pointer to the cloud of normals

    ROS_INFO("Calculando normais, aguarde um instante (o processo pode demorar)......");
    // Grab only coordinates from income cloud
    cloud_xyz_ptr->resize(recv_cloud_ptr->points.size()); // allocate memory space
    for(int i=0; i < recv_cloud_ptr->points.size(); i++){
      cloud_xyz_ptr->points[i].x = recv_cloud_ptr->points[i].x;
      cloud_xyz_ptr->points[i].y = recv_cloud_ptr->points[i].y;
      cloud_xyz_ptr->points[i].z = recv_cloud_ptr->points[i].z;
    }
    // Compute normals from coordinates data
    ne.setInputCloud(cloud_xyz_ptr);
//    normals_cloud_ptr->resize(cloud_xyz_ptr->points.size()); // allocate memory
    ne.compute(*normals_cloud_ptr);

    // Output cloud to save
    pcl::PointCloud<Out>::Ptr output_cloud_ptr (new pcl::PointCloud<Out> ());

    // Concatenate both data from income cloud plus normals and save a ply file
//    output_cloud_ptr->resize(recv_cloud_ptr->points.size()); // allocate memory
    ROS_INFO("Concatenando os campos......");
    pcl::concatenateFields(*recv_cloud_ptr, *normals_cloud_ptr, *output_cloud_ptr);

    pcl::io::savePLYFileASCII(filename, *output_cloud_ptr);
    ROS_INFO("Tudo correto, conferir pelo arquivo na area de trabalho !!");

  }
  // Kill the node after saving the ptcloud
  ros::shutdown();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "save_cloud");
  ros::NodeHandle nh;

  ROS_INFO("Iniciando o processo de salvar dados pos processados...");

  ros::Subscriber sub = nh.subscribe("/accumulated_point_cloud", 100, savecloud_plus_normal_ply);

  ros::spin();

  return 0;
}
