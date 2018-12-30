//Includes
#define PCL_NO_PRECOMPILE
#include <cmath>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/tf.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <mavros/mavros.h>
#include <mavros_msgs/Mavlink.h>
#include <mavros_msgs/mavlink_convert.h>
#include <mavlink/config.h>
#include <mavros/utils.h>
#include <mavros/mavros.h>
#include <mavros_msgs/mavlink_convert.h>
#include <mavros_msgs/VFR_HUD.h>
#include <mavros_msgs/GlobalPositionTarget.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <dynamixel_workbench_toolbox/dynamixel_multi_driver.h>
#include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <dynamixel_workbench_msgs/DynamixelState.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
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

/// Namespaces
using namespace pcl;
using namespace std;
using namespace message_filters;
using namespace nav_msgs;
using namespace mavros_msgs;
using namespace dynamixel_workbench_msgs;
using namespace geometry_msgs;
using namespace Eigen;

/// Definitions
typedef PointXYZRGB PointT;
typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, Odometry> syncPolicy;

//struct PointT
//{
//  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
//  PCL_ADD_RGB;
//  float l;
//  float o;
//  float p;
//  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
//} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

//POINT_CLOUD_REGISTER_POINT_STRUCT  (PointT,           // here we assume a XYZ + "test" (as fields)
//                                    (float, x, x)
//                                    (float, y, y)
//                                    (float, z, z)
//                                    (float, rgb, rgb)
//                                    //(float, g, g)
//                                    //(float, b, b)
//                                    (float, l, l)
//                                    (float, o, o)
//                                    (float, p, p)
//                                    )

/// Global vars
PointCloud<PointT >::Ptr accumulated_cloud;
PointCloud<PointT>::Ptr cloud_acumulada_termica;
tf::TransformListener *p_listener;
boost::shared_ptr<ros::Publisher> pub;
boost::shared_ptr<ros::Publisher> pub_termica;

float xmin = 1000000, xmax = -300000, xmintemp = 1000000, xmaxtemp = -300000;
float ymin = 1000000, ymax = -300000, ymintemp = 1000000, ymaxtemp = -300000;
float zmin = 1000000, zmax = -300000;

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
  VoxelGrid<PointT> grid;
  grid.setInputCloud(in);
  grid.setLeafSize(lf, lf, lf);
  grid.filter(*in);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void init_limits_xyz(PointCloud<PointT>::Ptr cloud){
  for(int i=0; i++; i<cloud->points.size()){
    if(cloud->points[i].x < xmin){
      xmin = cloud->points[i].x;
      xmintemp = cloud->points[i].x;
    }
    if(cloud->points[i].y < ymin){
      ymin = cloud->points[i].y;
      ymintemp = cloud->points[i].y;
    }
    if(cloud->points[i].x > xmax){
      xmax = cloud->points[i].x;
      xmaxtemp = cloud->points[i].x;
    }
    if(cloud->points[i].y > ymax){
      ymax = cloud->points[i].y;
      ymaxtemp = cloud->points[i].y;
    }
    if(cloud->points[i].z < zmin){
      zmin = cloud->points[i].z;
    }
    if(cloud->points[i].z > zmax){
      zmax = cloud->points[i].z;
    }
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void temp_limits_and_filter_xyz(PointCloud<PointT>::Ptr cloud){
  PointCloud<PointT>::iterator it = cloud->begin();
  for(it = cloud->begin(); it!=cloud->end(); it++){
    // Gather new limits if thats the case
    if(it->x < xmin){
      xmintemp = it->x;
    }
    if(it->y < ymin){
      ymintemp = it->y;
    }
    if(it->x > xmax){
      xmaxtemp = it->x;
    }
    if(it->y > ymax){
      ymaxtemp = it->y;
    }
    if(it->z < zmin){
      zmin = it->z;
    }
    if(it->z > zmax){
      zmax = it->z;
    }
    // If that is not the case, the cloud point can be filtered
    if(it->x > xmin && it->x < xmax && it->y > ymin && it->y < ymax){
      cloud->erase(it);
      it++;
    }
    if(it == cloud->end())
      return;
  }
  // Update the limits from the last cloud
  if(xmintemp < xmin)
    xmin = xmintemp;
  if(xmaxtemp > xmax)
    xmax = xmaxtemp;
  if(ymintemp < ymin)
    ymin = ymintemp;
  if(ymaxtemp > ymax)
    ymax = ymaxtemp;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void cloud_open_target(const sensor_msgs::PointCloud2ConstPtr& msg_ptc_vis,
//                       const sensor_msgs::PointCloud2ConstPtr& msg_ptc_ter,
                       const OdometryConstPtr& msg_odo){
  // Declare variables
  PointCloud<PointT>::Ptr cloud (new PointCloud<PointT>());
  PointCloud<PointT>::Ptr cloud_transformed (new PointCloud<PointT>());
//  PointCloud<PointT>::Ptr temp_termica (new PointCloud<PointT>());
  sensor_msgs::PointCloud2 msg_out;
//  sensor_msgs::PointCloud2 msg_termica_out;

  // Check for incorrect odometry from viso2
  if(msg_odo->pose.covariance.at(0) > 100){
    ROS_WARN("Nao se pode confiar na odometria, movimento rapido");
    return;
  }

  // Convert the ros message to pcl point cloud
  fromROSMsg (*msg_ptc_vis, *cloud);
//  fromROSMsg (*msg_ptc_ter, *temp_termica);

  // Remove any NAN points in the cloud
//  vector<int> indicesNAN;
//  removeNaNFromPointCloud(*cloud, *cloud, indicesNAN);
  // Voxel Grid
//  float lf = 0.05f;
//  filter_grid(cloud, lf);
  // Filter with passthrough filter -> region to see
  passthrough(cloud, "z",   0, 20);
  passthrough(cloud, "x", -10, 10);
  passthrough(cloud, "y", -10, 10);
  // Filter for color
//  filter_color(cloud);
  // Remove outiliers
  remove_outlier(cloud, 15, 0.2);
//  remove_outlier(temp_termica, 15, 0.2);

  //  // If first cloud, get the first limits
  //  if(accumulated_cloud->points.size() < 5){
  //    init_limits_xyz(cloud);
  //  } else {
  //    temp_limits_and_filter_xyz(cloud);
  //  }

  /// Obter a odometria da mensagem
  // Rotacao
  Eigen::Quaternion<double> q;
  q.x() = (double)msg_odo->pose.pose.orientation.x;
  q.y() = (double)msg_odo->pose.pose.orientation.y;
  q.z() = (double)msg_odo->pose.pose.orientation.z;
  q.w() = (double)msg_odo->pose.pose.orientation.w;
  // Translacao
  Eigen::Vector3d offset(msg_odo->pose.pose.position.x, msg_odo->pose.pose.position.y, msg_odo->pose.pose.position.z);

  // Transformar a nuvem
  transformPointCloud<PointT>(*cloud, *cloud_transformed, offset, q);
//  transformPointCloud<PointT>(*temp_termica, *temp_termica     , offset, q);

  // Accumulate the point cloud using the += operator
  (*accumulated_cloud) += (*cloud_transformed);
//  (*cloud_acumulada_termica) += (*temp_termica)     ;

  ROS_INFO("Tamanho da nuvem acumulada = %ld", accumulated_cloud->points.size());
//  ROS_INFO("Tamanho da nuvem termica   = %ld", cloud_acumulada_termica->points.size());

//  temp_termica->resize(cloud_transformed->size());
  // Trazendo a informacao termica, filtrando regiao que nao existe e acumulando
//  PoinT point;
//  for(int i=0; i<cloud_transformed->size(); i++)
//  {
//    if(cloud_transformed->points[i].l > 1 && cloud_transformed->points[i].o > 1 && cloud_transformed->points[i].p > 1){
//      point.x = cloud_transformed->points[i].x;
//      point.y = cloud_transformed->points[i].y;
//      point.z = cloud_transformed->points[i].z;

//      point.r = int(cloud_transformed->points[i].l);
//      point.g = int(cloud_transformed->points[i].o);
//      point.b = int(cloud_transformed->points[i].p);

//      temp_termica->push_back(point);
//      temp_termica->points[i].x = cloud_transformed->points[i].x;
//      temp_termica->points[i].y = cloud_transformed->points[i].y;
//      temp_termica->points[i].z = cloud_transformed->points[i].z;

//      temp_termica->points[i].r = int(cloud_transformed->points[i].l);
//      temp_termica->points[i].g = int(cloud_transformed->points[i].o);
//      temp_termica->points[i].b = int(cloud_transformed->points[i].p);
//    } else {
//      temp_termica->points[i].x = nan("");
//      temp_termica->points[i].y = nan("");
//      temp_termica->points[i].z = nan("");

//      temp_termica->points[i].r = nan("");
//      temp_termica->points[i].g = nan("");
//      temp_termica->points[i].b = nan("");
//    }
//  }

  // Limpar point cloud termica de elementos NAN
//  removeNaNFromPointCloud(*temp_termica, *temp_termica, indicesNAN);

  // Acumular termica sem filtros a principio
//  (*cloud_acumulada_termica) += (*temp_termica);

  // Convert the pcl point cloud to ros msg and publish
  toROSMsg(*accumulated_cloud, msg_out);
  msg_out.header.stamp = ros::Time::now();
  pub->publish(msg_out);
//  toROSMsg(*cloud_acumulada_termica, msg_termica_out);
//  msg_termica_out.header.stamp = ros::Time::now();
//  pub_termica->publish(msg_termica_out);

  cloud.reset();
  cloud_transformed.reset();
//  temp_termica.reset();

}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "accumulatepointcloud");
  ros::NodeHandle nh;

  // Initialize accumulated cloud variable
  accumulated_cloud = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;
  accumulated_cloud->header.frame_id = "odom";
//  cloud_acumulada_termica = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;
//  cloud_acumulada_termica->header.frame_id = "odom";

  // Initialize the point cloud publisher
  pub = (boost::shared_ptr<ros::Publisher>) new ros::Publisher;
  *pub = nh.advertise<sensor_msgs::PointCloud2>("/accumulated_point_cloud", 300);
//  pub_termica = (boost::shared_ptr<ros::Publisher>) new ros::Publisher;
//  *pub_termica = nh.advertise<sensor_msgs::PointCloud2>("/accumulated_termica", 300);

  // Subscriber para a nuvem instantanea e odometria
  message_filters::Subscriber<sensor_msgs::PointCloud2>  subptcvis(nh, "/overlap/visual_cloud"    , 100);
//  message_filters::Subscriber<sensor_msgs::PointCloud2>  subptcter(nh, "/termica/termica_pc"      , 100); // ALTEREI O TAMANHO DAS FILAS!
  message_filters::Subscriber<Odometry>                  subodo   (nh, "/overlap/odometry", 100);

  // Sincroniza as leituras dos topicos (sensores e imagem a principio) em um so callback
  Synchronizer<syncPolicy> sync(syncPolicy(100), subptcvis, subodo); // ALTEREI O TAMANHO DAS FILAS!
  sync.registerCallback(boost::bind(&cloud_open_target, _1, _2));

  // Loop infinitely
//  ros::Rate rate(10);
//  while(ros::ok()){
//    ros::spinOnce();
//    rate.sleep();
//  }
  ros::spin();

  return 0;

}
