//Includes
#define PCL_NO_PRECOMPILE
#include <cmath>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace pcl;
using namespace std;
using namespace message_filters;
using namespace nav_msgs;

//Definitions
typedef PointXYZRGB PointTT;
typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, Odometry> syncPolicy;

struct PointT
{
  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  PCL_ADD_RGB;
  float l;
  float o;
  float p;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT  (PointT,           // here we assume a XYZ + "test" (as fields)
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

//Global vars
PointCloud<PointT>::Ptr accumulated_cloud;
tf::TransformListener *p_listener;
boost::shared_ptr<ros::Publisher> pub;
boost::shared_ptr<ros::Publisher> pub_termica;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void filter_color(PointCloud<PointT>::Ptr cloud_in){

  // Try to clear white and blue points from sky, and green ones from the grass, or something close to it
  int rMax = 210;
  int rMin = 0;
  int gMax = 210;
  int gMin = 0;
  int bMax = 210;
  int bMin = 0;

  ConditionAnd<PointT>::Ptr color_cond (new ConditionAnd<PointT> ());
  color_cond->addComparison (PackedRGBComparison<PointT>::Ptr (new PackedRGBComparison<PointT> ("r", ComparisonOps::LT, rMax)));
  color_cond->addComparison (PackedRGBComparison<PointT>::Ptr (new PackedRGBComparison<PointT> ("r", ComparisonOps::GT, rMin)));
  color_cond->addComparison (PackedRGBComparison<PointT>::Ptr (new PackedRGBComparison<PointT> ("g", ComparisonOps::LT, gMax)));
  color_cond->addComparison (PackedRGBComparison<PointT>::Ptr (new PackedRGBComparison<PointT> ("g", ComparisonOps::GT, gMin)));
  color_cond->addComparison (PackedRGBComparison<PointT>::Ptr (new PackedRGBComparison<PointT> ("b", ComparisonOps::LT, bMax)));
  color_cond->addComparison (PackedRGBComparison<PointT>::Ptr (new PackedRGBComparison<PointT> ("b", ComparisonOps::GT, bMin)));

  // build the filter
  ConditionalRemoval<PointT> condrem (color_cond);
  condrem.setInputCloud (cloud_in);
  condrem.setKeepOrganized(true);

  // apply filter
  condrem.filter (*cloud_in);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void passthrough(PointCloud<PointT>::Ptr in, std::string field, float min, float max){
  PassThrough<PointT> ps;
  ps.setInputCloud(in);
  ps.setFilterFieldName(field);
  ps.setFilterLimits(min, max);

  ps.filter(*in);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void remove_outlier(PointCloud<PointT>::Ptr in, float mean, float deviation){
  StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud(in);
  sor.setMeanK(mean);
  sor.setStddevMulThresh(deviation);
  sor.filter(*in);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void filter_grid(PointCloud<PointT>::Ptr in, float lf){
  VoxelGrid<PointTT> grid;
  grid.setInputCloud(in);
  grid.setLeafSize(lf, lf, lf);
  grid.filter(*in);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void cloud_open_target2(const sensor_msgs::PointCloud2ConstPtr& msg_ptc, const OdometryConstPtr& msg_odo){
  // Declare variables
  PointCloud<PointT>::Ptr cloud (new PointCloud<PointT>());
  PointCloud<PointT>::Ptr cloud_transformed (new PointCloud<PointT>());
//  VoxelGrid<PointT> grid;
  sensor_msgs::PointCloud2 msg_out;

  PointCloud<PointTT> cloud_acumulada_termica;
//  sensor_msgs::PointCloud2 visual_out;
  sensor_msgs::PointCloud2 termica_out;

  // Convert the ros message to pcl point cloud
  fromROSMsg (*msg_ptc, *cloud);

  // Remove any NAN points in the cloud
  vector<int> indicesNAN;
  removeNaNFromPointCloud(*cloud, *cloud, indicesNAN);
  // Voxel Grid
  float lf = 0.05f;
  filter_grid(cloud, lf);
  // Filter with passthrough filter -> region to see
  passthrough(cloud, "z",   1, 35);
  passthrough(cloud, "x", -10, 10);
  passthrough(cloud, "y", -10, 10);
  // Filter for color
  filter_color(cloud);
  // Remove outiliers
  remove_outlier(cloud, 19, 0.5);

  /// Obter a odometria da mensagem
  // Rotacao
  Eigen::Quaternion<double> q;
  q.x() = (double)msg_odo->pose.pose.orientation.x;
  q.y() = (double)msg_odo->pose.pose.orientation.y;
  q.z() = (double)msg_odo->pose.pose.orientation.z;
  q.w() = (double)msg_odo->pose.pose.orientation.w;
  // Translacao
  Eigen::Vector3d offset(msg_odo->pose.pose.position.x, msg_odo->pose.pose.position.y, msg_odo->pose.pose.position.z);
  // Print para averiguar
  if(true){
//    auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
//    cout << "Roll: " << RAD2DEG(euler[0]) << "\tPitch: " << RAD2DEG(euler[1]) << "\tYaw: " << RAD2DEG(euler[2]) << endl;
    cout << "X   : " << offset(0)         << "\tY    : " << offset(1)         << "\tZ  : " << offset(2)         << endl;
  }

  // Transformar a nuvem
  transformPointCloud<PointT>(*cloud, *cloud_transformed, offset, q);

//  //Get the transform, return if cannot get it
//  ros::Time tic = ros::Time::now();
//  ros::Time t = msg->header.stamp;
//  tf::StampedTransform trans;
//  try
//  {
//    p_listener->waitForTransform(ros::names::remap("/map"), msg->header.frame_id, t, ros::Duration(3.0));
//    p_listener->lookupTransform(ros::names::remap("/map"), msg->header.frame_id, t, trans);
//  }
//  catch (tf::TransformException& ex){
//    ROS_ERROR("%s",ex.what());
//    //ros::Duration(1.0).sleep();
//    ROS_WARN("Cannot accumulate");
//    return;
//  }
//  ROS_INFO("Collected transforms (%0.3f secs)", (ros::Time::now() - tic).toSec());

//  //Transform point cloud using the transform obtained
//  Eigen::Affine3d eigen_trf;
//  tf::transformTFToEigen (trans, eigen_trf);
//  transformPointCloud<PointT>(*cloud, *cloud_transformed, eigen_trf);

  // Accumulate the point cloud using the += operator
  ROS_INFO("Size of cloud_transformed = %ld", cloud_transformed->points.size());
  (*accumulated_cloud) += (*cloud_transformed);

  ROS_INFO("Size of accumulated_cloud = %ld", accumulated_cloud->points.size());

  // Separando as point clouds em visual e térmica
  int nPontos = int(accumulated_cloud->points.size());
//  cloud_acumulada_visual.points.resize (nPontos);
  cloud_acumulada_termica.points.resize (nPontos);
  for(int i = 0; i < int(accumulated_cloud->points.size()); i++)
  {

//      cloud_acumulada_visual.points[i].x = accumulated_cloud->points[i].x;
//      cloud_acumulada_visual.points[i].y = accumulated_cloud->points[i].y;
//      cloud_acumulada_visual.points[i].z = accumulated_cloud->points[i].z;

//      cloud_acumulada_visual.points[i].r = accumulated_cloud->points[i].r;
//      cloud_acumulada_visual.points[i].g = accumulated_cloud->points[i].g;
//      cloud_acumulada_visual.points[i].b = accumulated_cloud->points[i].b;

      cloud_acumulada_termica.points[i].x = accumulated_cloud->points[i].x;
      cloud_acumulada_termica.points[i].y = accumulated_cloud->points[i].y;
      cloud_acumulada_termica.points[i].z = accumulated_cloud->points[i].z;

      cloud_acumulada_termica.points[i].r = accumulated_cloud->points[i].l;
      cloud_acumulada_termica.points[i].g = accumulated_cloud->points[i].o;
      cloud_acumulada_termica.points[i].b = accumulated_cloud->points[i].p;\

      if(cloud_acumulada_termica.points[i].r != -1)
      {
          cloud_acumulada_termica.points[i].r = accumulated_cloud->points[i].l;
          cloud_acumulada_termica.points[i].g = accumulated_cloud->points[i].o;
          cloud_acumulada_termica.points[i].b = accumulated_cloud->points[i].p;\
      }
      else
      {
          cloud_acumulada_termica.points[i].x = nan("");
          cloud_acumulada_termica.points[i].y = nan("");
          cloud_acumulada_termica.points[i].z = nan("");
          cloud_acumulada_termica.points[i].r = nan("");
          cloud_acumulada_termica.points[i].g = nan("");
          cloud_acumulada_termica.points[i].b = nan("");
      }

  }

  // Limpar point cloud termica de elementos NAN
  vector<int> indicesNAN2;
  removeNaNFromPointCloud(cloud_acumulada_termica, cloud_acumulada_termica, indicesNAN2);

  //Convert the pcl point cloud to ros msg and publish
  toROSMsg(*accumulated_cloud, msg_out);
  msg_out.header.stamp = msg_ptc->header.stamp;
  pub->publish(msg_out);

  // Publicando point cloud visual e térmica
//  toROSMsg (cloud_acumulada_visual, visual_out);
  toROSMsg (cloud_acumulada_termica, termica_out);
//  visual_out.header.stamp = t;
//  visual_out.header.frame_id = ros::names::remap("/map");
  termica_out.header.stamp = msg_ptc->header.stamp;
  termica_out.header.frame_id = ros::names::remap("/map");
//  pub_visual->publish(visual_out);
  pub_termica->publish(termica_out);

  cloud.reset();
  cloud_transformed.reset();
}


void cloud_open_target(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  //declare variables
  PointCloud<PointT>::Ptr cloud;
  PointCloud<PointT>::Ptr cloud_transformed;
  VoxelGrid<PointT> grid;
  sensor_msgs::PointCloud2 msg_out;

  PointCloud<PointTT> cloud_acumulada_visual;
  PointCloud<PointTT> cloud_acumulada_termica;
  sensor_msgs::PointCloud2 visual_out;
  sensor_msgs::PointCloud2 termica_out;

  //allocate objects for pointers
  cloud = (PointCloud<PointT>::Ptr) (new PointCloud<PointT>);
  cloud_transformed = (PointCloud<PointT>::Ptr) (new PointCloud<PointT>);

  //Convert the ros message to pcl point cloud
  fromROSMsg (*msg, *cloud);

  //Remove any NAN points in the cloud
  std::vector<int> indicesNAN;
  removeNaNFromPointCloud(*cloud, *cloud, indicesNAN);
  // Filter with passthrough filter -> region to see
//  passthrough(cloud, "z", 1, 35);
  passthrough(cloud, "x", -10, 10);
  passthrough(cloud, "y", -10, 10);
  // Filter for color
  filter_color(cloud);
  // Remove outiliers
  remove_outlier(cloud, 19, 0.5);
//  // Voxel grid
//  grid.setInputCloud(cloud);
//  grid.setLeafSize(lf, lf, lf);
//  grid.filter(*cloud);
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
  transformPointCloud<PointT>(*cloud, *cloud_transformed, eigen_trf);

  //Accumulate the point cloud using the += operator
  ROS_INFO("Size of cloud_transformed = %ld", cloud_transformed->points.size());
  (*accumulated_cloud) += (*cloud_transformed);

  ROS_INFO("Size of accumulated_cloud = %ld", accumulated_cloud->points.size());

  // Separando as point clouds em visual e térmica
  int nPontos = int(accumulated_cloud->points.size());
  cloud_acumulada_visual.points.resize (nPontos);
  cloud_acumulada_termica.points.resize (nPontos);
  for(int i = 0; i < int(accumulated_cloud->points.size()); i++)
  {


      cloud_acumulada_visual.points[i].x = accumulated_cloud->points[i].x;
      cloud_acumulada_visual.points[i].y = accumulated_cloud->points[i].y;
      cloud_acumulada_visual.points[i].z = accumulated_cloud->points[i].z;

      cloud_acumulada_visual.points[i].r = accumulated_cloud->points[i].r;
      cloud_acumulada_visual.points[i].g = accumulated_cloud->points[i].g;
      cloud_acumulada_visual.points[i].b = accumulated_cloud->points[i].b;

      cloud_acumulada_termica.points[i].x = accumulated_cloud->points[i].x;
      cloud_acumulada_termica.points[i].y = accumulated_cloud->points[i].y;
      cloud_acumulada_termica.points[i].z = accumulated_cloud->points[i].z;

      cloud_acumulada_termica.points[i].r = accumulated_cloud->points[i].l;
      cloud_acumulada_termica.points[i].g = accumulated_cloud->points[i].o;
      cloud_acumulada_termica.points[i].b = accumulated_cloud->points[i].p;\

      if(cloud_acumulada_termica.points[i].r != -1)
      {
          cloud_acumulada_termica.points[i].r = accumulated_cloud->points[i].l;
          cloud_acumulada_termica.points[i].g = accumulated_cloud->points[i].o;
          cloud_acumulada_termica.points[i].b = accumulated_cloud->points[i].p;\
      }
      else
      {
          cloud_acumulada_termica.points[i].x = nan("");
          cloud_acumulada_termica.points[i].y = nan("");
          cloud_acumulada_termica.points[i].z = nan("");
          cloud_acumulada_termica.points[i].r = nan("");
          cloud_acumulada_termica.points[i].g = nan("");
          cloud_acumulada_termica.points[i].b = nan("");
      }

  }

  // Limpar point cloud termica de elementos NAN
  std::vector<int> indicesNAN2;
  removeNaNFromPointCloud(cloud_acumulada_termica, cloud_acumulada_termica, indicesNAN2);

  //Conver the pcl point cloud to ros msg and publish
  toROSMsg (*accumulated_cloud, msg_out);
  msg_out.header.stamp = t;
  pub->publish(msg_out);

  // Publicando point cloud visual e térmica
//  toROSMsg (cloud_acumulada_visual, visual_out);
  toROSMsg (cloud_acumulada_termica, termica_out);
  visual_out.header.stamp = t;
  visual_out.header.frame_id = ros::names::remap("/map");
  termica_out.header.stamp = t;
  termica_out.header.frame_id = ros::names::remap("/map");
//  pub_visual->publish(visual_out);
  pub_termica->publish(termica_out);

  cloud.reset();
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
  accumulated_cloud = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;
  accumulated_cloud->header.frame_id = ros::names::remap("/map");

  //Initialize the point cloud publisher
  pub = (boost::shared_ptr<ros::Publisher>) new ros::Publisher;
  *pub = nh.advertise<sensor_msgs::PointCloud2>("/accumulated_point_cloud", 1);
//  pub_visual = (boost::shared_ptr<ros::Publisher>) new ros::Publisher;
//  *pub_visual = nh.advertise<sensor_msgs::PointCloud2>("/accumulated_visual", 1);
  pub_termica = (boost::shared_ptr<ros::Publisher>) new ros::Publisher;
  *pub_termica = nh.advertise<sensor_msgs::PointCloud2>("/accumulated_termica", 1);

  // Subscriber para a nuvem instantanea e odometria
  message_filters::Subscriber<sensor_msgs::PointCloud2> subptc(nh, "/completa_pc"             , 100);
  message_filters::Subscriber<Odometry>                 subodo(nh, "/stereo_odometer/odometry", 100);
  // Sincroniza as leituras dos topicos (sensores e imagem a principio) em um so callback
  Synchronizer<syncPolicy> sync(syncPolicy(100), subptc, subodo);
  sync.registerCallback(boost::bind(&cloud_open_target2, _1, _2));

  // Create a ROS subscriber for the input point cloud  -- Se inscreve na point cloud gerada no pkg termica_reconstrucao
//  ros::Subscriber sub_target = nh.subscribe ("/completa_pc", 10, cloud_open_target);

  //Loop infinitly
  ros::spin();
//  while (ros::ok())
//  {
//    // Spin
//    ros::spinOnce();
//  }
  //Save accumulated point cloud to a file
//  printf("Saving to file %s\n", filename.c_str());
//  io::savePCDFileASCII (filename, *accumulated_cloud);

  return 0;

}
