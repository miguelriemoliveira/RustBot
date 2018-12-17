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

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace message_filters;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> syncPolicy;

// FIle to save
std::string filename;
// Define to simplify matters
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZRGBNormal Out;
// Control of the saved clouds
bool visual_gravada = false, termica_gravada = false;

struct PointTT
{
  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  PCL_ADD_RGB;
  float l;
  float o;
  float p;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT  (PointTT,           // here we assume a XYZ + "test" (as fields)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, rgb, rgb)
                                   //(float, g, g)
                                   //(float, b, b)
                                   (float, l, l)
                                   (float, o, o)
                                   (float, p, p)
)

void save_acumulada_plus_normal_ply(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  ROS_INFO("Recebendo dados visuais para salvar......");
  // Declare the pointer to the received cloud
  pcl::PointCloud<Out>::Ptr recv_cloud_ptr(new pcl::PointCloud<Out>); // Allocate memory

  // Pass the message to a pcl entity
  pcl::fromROSMsg(*msg, *recv_cloud_ptr);

  std::vector<int> indicesNAN;
  pcl::removeNaNFromPointCloud(*recv_cloud_ptr, *recv_cloud_ptr, indicesNAN);

//  // Clouds to pass info around
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_ptr (new pcl::PointCloud<pcl::PointXYZ>); // Gather only pose so can compute normals
//  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne; // Entity to compute normals in the future
//  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ()); // Kd Tree used to divide space and compute normals
//  ne.setSearchMethod (tree);
//  ne.setKSearch(8);
//  pcl::PointCloud<pcl::Normal>::Ptr normals_cloud_ptr (new pcl::PointCloud<pcl::Normal> ()); // Pointer to the cloud of normals

//  // Output cloud to save
//  pcl::PointCloud<Out>::Ptr output_cloud_ptr (new pcl::PointCloud<Out> ());

//  ROS_INFO("Calculando normais, aguarde um instante (o processo pode demorar)......");
//  // Grab only coordinates from income cloud
//  cloud_xyz_ptr->resize(recv_cloud_ptr->points.size()); // allocate memory space
//  for(int i=0; i < recv_cloud_ptr->points.size(); i++){
//    cloud_xyz_ptr->points[i].x = recv_cloud_ptr->points[i].x;
//    cloud_xyz_ptr->points[i].y = recv_cloud_ptr->points[i].y;
//    cloud_xyz_ptr->points[i].z = recv_cloud_ptr->points[i].z;
//  }
//  // Compute normals from coordinates data
//  ne.setInputCloud(cloud_xyz_ptr);
//  normals_cloud_ptr->resize(cloud_xyz_ptr->points.size()); // allocate memory
//  ne.compute(*normals_cloud_ptr);

//  // Concatenate both data from income cloud plus normals and save a ply file
//  output_cloud_ptr->resize(recv_cloud_ptr->points.size()); // allocate memory
//  ROS_INFO("Concatenando os campos......");
//  pcl::concatenateFields(*recv_cloud_ptr, *normals_cloud_ptr, *output_cloud_ptr);

  //////////////////////////// SALVAR
  // Hora atual
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
  std::string date = "_" + year + "_" + month + "_" + day + "_" + hour + "h_" + minutes + "m.ply";

  // Salvando
  filename = "/home/mrs/Desktop/pos_processo_visual_em"+date;
  pcl::io::savePLYFileASCII(filename, *recv_cloud_ptr);

  ROS_INFO("Tudo correto, conferir pelos arquivos na area de trabalho !!");

  // Kill the node after saving the ptcloud
  visual_gravada = true;
  termica_gravada = true;
  ros::shutdown();
}

void savetermica_ply(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  ROS_INFO("Recebendo dados termicos para salvar......");
  // Declare the pointer to the received cloud
  pcl::PointCloud<PointTT>::Ptr recv_cloud_ptr(new pcl::PointCloud<PointTT>); // Allocate memory

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
  std::string date = "_" + year + "_" + month + "_" + day + "_" + hour + "h_" + minutes + "m.ply";
  filename = "/home/mrs/Desktop/pos_processo_termico_em"+date;
  pcl::io::savePLYFileASCII(filename, *recv_cloud_ptr);
  ROS_INFO("Tudo correto, conferir pelo arquivo na area de trabalho !!");

  termica_gravada = true;
}

void savecloud_plus_normal_ply(const sensor_msgs::PointCloud2ConstPtr& msg, const sensor_msgs::PointCloud2ConstPtr& msgt)
{
  //////////////////////////// VISUAL

  ROS_INFO("Recebendo dados para salvar......");
  // Declare the pointer to the received cloud
  pcl::PointCloud<PointT>::Ptr recv_cloud_ptr(new pcl::PointCloud<PointT>); // Allocate memory

  // Pass the message to a pcl entity
  pcl::fromROSMsg(*msg, *recv_cloud_ptr);

  // Clouds to pass info around
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_ptr (new pcl::PointCloud<pcl::PointXYZ>); // Gather only pose so can compute normals
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne; // Entity to compute normals in the future
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ()); // Kd Tree used to divide space and compute normals
  ne.setSearchMethod (tree);
  ne.setKSearch(5);
  pcl::PointCloud<pcl::Normal>::Ptr normals_cloud_ptr (new pcl::PointCloud<pcl::Normal> ()); // Pointer to the cloud of normals

  // Output cloud to save
  pcl::PointCloud<Out>::Ptr output_cloud_ptr (new pcl::PointCloud<Out> ());

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
  normals_cloud_ptr->resize(cloud_xyz_ptr->points.size()); // allocate memory
  ne.compute(*normals_cloud_ptr);

  // Concatenate both data from income cloud plus normals and save a ply file
  output_cloud_ptr->resize(recv_cloud_ptr->points.size()); // allocate memory
  ROS_INFO("Concatenando os campos......");
  pcl::concatenateFields(*recv_cloud_ptr, *normals_cloud_ptr, *output_cloud_ptr);

  //////////////////////////// TERMICA

  ROS_INFO("Recebendo dados termicos para salvar......");
  // Declare the pointer to the received cloud
  pcl::PointCloud<PointTT>::Ptr recv_cloud_term_ptr(new pcl::PointCloud<PointTT>); // Allocate memory

  // Pass the message to a pcl entity
  pcl::fromROSMsg(*msgt, *recv_cloud_term_ptr);

  //////////////////////////// SALVAR
  // Hora atual
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
  std::string date = "_" + year + "_" + month + "_" + day + "_" + hour + "h_" + minutes + "m.ply";

  // Salvando
  filename = "/home/mrs/Desktop/pos_processo_visual_em"+date;
  pcl::io::savePLYFileASCII(filename, *output_cloud_ptr);
  filename = "/home/mrs/Desktop/pos_processo_termico_em"+date;
  pcl::io::savePLYFileASCII(filename, *recv_cloud_term_ptr);

  ROS_INFO("Tudo correto, conferir pelos arquivos na area de trabalho !!");

  // Kill the node after saving the ptcloud
  visual_gravada = true;
  termica_gravada = true;
  ros::shutdown();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "save_cloud");
  ros::NodeHandle nh;

  ROS_INFO("Iniciando o processo de salvar dados pos processados...");

  ros::Subscriber sub  = nh.subscribe("/accumulated_point_cloud", 1000, save_acumulada_plus_normal_ply);
//  ros::Subscriber subt = nh.subscribe("/accumulated_termica", 1000, savetermica_ply);

//  message_filters::Subscriber<sensor_msgs::PointCloud2>  sub(nh, "/accumulated_point_cloud", 100);
//  message_filters::Subscriber<sensor_msgs::PointCloud2> subt(nh, "/accumulated_termica"    , 100);

//   Sincroniza as leituras dos topicos (sensores a principio) em um so callback
//  Synchronizer<syncPolicy> sync(syncPolicy(100), sub, subt);
//  sync.registerCallback(boost::bind(&savecloud_plus_normal_ply, _1, _2));

  while(true){
    ros::spinOnce();
    if(visual_gravada && termica_gravada){
      ros::shutdown();
      break;
    }
  }
  ros::shutdown();

  return 0;
}
