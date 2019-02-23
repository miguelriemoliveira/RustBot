#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>

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
// Accumulated clouds to save as global, and write .ply on desktop when the time is right
pcl::PointCloud<PointT>::Ptr termica_acumulada;
pcl::PointCloud<PointT>::Ptr visual_acumulada;

void save_acumulada_plus_normal_ply(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  if(!visual_gravada){

    pcl::PointCloud<PointT>::Ptr recv_cloud_ptr(new pcl::PointCloud<PointT>); // Allocate memory
    pcl::fromROSMsg(*msg, *recv_cloud_ptr);
    if(recv_cloud_ptr->size() > 10)
      *visual_acumulada = *recv_cloud_ptr;
    ROS_INFO("Nuvem visual atualizada e apta para ser salva.");
//    ROS_INFO("Recebendo dados visuais para salvar......");
//    // Declare the pointer to the received cloud
//    pcl::PointCloud<PointT>::Ptr recv_cloud_ptr(new pcl::PointCloud<PointT>); // Allocate memory

//    // Pass the message to a pcl entity
//    pcl::fromROSMsg(*msg, *recv_cloud_ptr);

//    std::vector<int> indicesNAN;
//    pcl::removeNaNFromPointCloud(*recv_cloud_ptr, *recv_cloud_ptr, indicesNAN);

//    //////////////////////////// SALVAR
//    // Hora atual
//    ROS_INFO("Salvando o arquivo .ply na area de trabalho......");
//    // Ver o tempo para diferenciar bags gravadas automaticamente
//    time_t t = time(0);
//    struct tm * now = localtime( & t );
//    std::string year, month, day, hour, minutes, home;
//    char const* tmp = getenv("HOME");
//    if(tmp)
//      home = std::string(tmp);;
//    year    = boost::lexical_cast<std::string>(now->tm_year + 1900);
//    month   = boost::lexical_cast<std::string>(now->tm_mon );
//    day     = boost::lexical_cast<std::string>(now->tm_mday);
//    hour    = boost::lexical_cast<std::string>(now->tm_hour);
//    minutes = boost::lexical_cast<std::string>(now->tm_min );
//    std::string date = "_" + year + "_" + month + "_" + day + "_" + hour + "h_" + minutes + "m.ply";

//    // Salvando
//    filename = home+"/Desktop/pos_processo_visual_em"+date;
//    pcl::io::savePLYFileASCII(filename, *recv_cloud_ptr);

//    ROS_INFO("Tudo correto, conferir pelos arquivos na area de trabalho !!");

//    // Kill the node after saving the ptcloud
//    visual_gravada = true;
//  }
//  if(visual_gravada && termica_gravada){
//    ros::shutdown();
  }
}

void savetermica_ply(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  if(!termica_gravada){

    pcl::PointCloud<PointT>::Ptr recv_cloud_ptr(new pcl::PointCloud<PointT>); // Allocate memory
    pcl::fromROSMsg(*msg, *recv_cloud_ptr);
    if(recv_cloud_ptr->size() > 10)
      *termica_acumulada = *recv_cloud_ptr;
    ROS_INFO("Nuvem termica atualizada e apta para ser salva.");
//    ROS_INFO("Recebendo dados termicos para salvar......");
//    // Declare the pointer to the received cloud
//    pcl::PointCloud<PointT>::Ptr recv_cloud_ptr(new pcl::PointCloud<PointT>); // Allocate memory

//    // Pass the message to a pcl entity
//    pcl::fromROSMsg(*msg, *recv_cloud_ptr);


//    ROS_INFO("Salvando o arquivo .ply na area de trabalho......");
//    // Ver o tempo para diferenciar bags gravadas automaticamente
//    time_t t = time(0);
//    struct tm * now = localtime( & t );
//    std::string year, month, day, hour, minutes, home;
//    char const* tmp = getenv("HOME");
//    if(tmp)
//      home = std::string(tmp);
//    year    = boost::lexical_cast<std::string>(now->tm_year + 1900);
//    month   = boost::lexical_cast<std::string>(now->tm_mon );
//    day     = boost::lexical_cast<std::string>(now->tm_mday);
//    hour    = boost::lexical_cast<std::string>(now->tm_hour);
//    minutes = boost::lexical_cast<std::string>(now->tm_min );
//    std::string date = "_" + year + "_" + month + "_" + day + "_" + hour + "h_" + minutes + "m.ply";
//    filename = home+"/Desktop/pos_processo_termico_em"+date;
//    pcl::io::savePLYFileASCII(filename, *recv_cloud_ptr);
//    ROS_INFO("Tudo correto, conferir pelo arquivo na area de trabalho !!");

//    termica_gravada = true;
//  }
//  if(visual_gravada && termica_gravada){
//    ros::shutdown();
  }
}

void salvando_nuvens(const std_msgs::BoolConstPtr& salvar){
  if(salvar->data == true){
    // Filtrando nuvens
    std::vector<int> indicesNAN;
    pcl::removeNaNFromPointCloud(*visual_acumulada, *visual_acumulada, indicesNAN);
    std::vector<int> indicesNAN2;
    pcl::removeNaNFromPointCloud(*termica_acumulada, *visual_acumulada, indicesNAN2);

    // Ver o tempo para diferenciar bags gravadas automaticamente
    time_t t = time(0);
    struct tm * now = localtime( & t );
    std::string year, month, day, hour, minutes, home;
    char const* tmp = getenv("HOME");
    if(tmp)
      home = std::string(tmp);
    year    = boost::lexical_cast<std::string>(now->tm_year + 1900);
    month   = boost::lexical_cast<std::string>(now->tm_mon );
    day     = boost::lexical_cast<std::string>(now->tm_mday);
    hour    = boost::lexical_cast<std::string>(now->tm_hour);
    minutes = boost::lexical_cast<std::string>(now->tm_min );
    std::string date = "_" + year + "_" + month + "_" + day + "_" + hour + "h_" + minutes + "m.ply";
    ROS_INFO("Recebendo dados termicos para salvar......");

    if(termica_acumulada->size() > 0){
    ROS_INFO("Salvando o arquivo .ply na area de trabalho......");

    filename = home+"/Desktop/pos_processo_termico_em"+date;
    pcl::io::savePLYFileASCII(filename, *termica_acumulada);
    ROS_INFO("Tudo correto, conferir pelo arquivo na area de trabalho !!");

    termica_gravada = true;
    } else {
      ROS_INFO("Nao ha dados termicos, nada foi salvo...");
    }

    ROS_INFO("Recebendo dados visuais para salvar......");

    if(visual_acumulada->size() > 0){
    ROS_INFO("Salvando o arquivo .ply na area de trabalho......");
    // Salvando
    filename = home+"/Desktop/pos_processo_visual_em"+date;
    pcl::io::savePLYFileASCII(filename, *visual_acumulada);

    ROS_INFO("Tudo correto, conferir pelos arquivos na area de trabalho !!");

    // Kill the node after saving the ptcloud
    visual_gravada = true;
    } else {
      ROS_INFO("Nao ha dados visuais, nada foi salvo...");
    }
  }
  if(visual_gravada && termica_gravada)
    ros::shutdown();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "save_cloud");
  ros::NodeHandle nh;

  ROS_INFO("Iniciando o processo de salvar dados pos processados...");

  visual_acumulada  = (pcl::PointCloud<PointT>::Ptr) new pcl::PointCloud<PointT>;
  termica_acumulada = (pcl::PointCloud<PointT>::Ptr) new pcl::PointCloud<PointT>;

  ros::Subscriber sub     = nh.subscribe("/accumulated_point_cloud", 1000, save_acumulada_plus_normal_ply);
  ros::Subscriber subt    = nh.subscribe("/accumulated_termica", 1000, savetermica_ply);
  ros::Subscriber subbool = nh.subscribe("/podemos_salvar_nuvens", 1000, salvando_nuvens);

  while(ros::ok()){
    ros::spinOnce();
    if(visual_gravada && termica_gravada){
      ros::shutdown();
      break;
    }
  }
  ros::shutdown();

  return 0;
}
