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
typedef PointXYZRGB PointTT;
typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, Odometry, VFR_HUD, Odometry, Odometry> syncPolicy;

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

/// Global vars
PointCloud<PointTT>::Ptr accumulated_cloud;
tf::TransformListener *p_listener;
boost::shared_ptr<ros::Publisher> pub;
boost::shared_ptr<ros::Publisher> pub_termica;
// Motors
double pan_front, pan_current , pan_previous , pwm2pan ; // YAW   [PWM, PWM, DEG/PWM]
double tilt_hor , tilt_current, tilt_previous, pwm2tilt; // PITCH [PWM, PWM, DEG/PWM]
double vel_pan_previous, vel_tilt_previous, vel_pan_current, vel_tilt_current; // quanto de PWM esta desviando
// Mavros
bool   first_read_mavros_dyn;
double yaw_offset_board; // [RAD]
double east_offset, north_offset; // [m]
double yaw_current_board, north_current, north_previous, east_current, east_previous;
// Odometria resultante
ros::Publisher pubodom;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void filter_color(PointCloud<PointTT>::Ptr cloud_in){

  // Try to clear white and blue points from sky, and green ones from the grass, or something close to it
  int rMax = 210;
  int rMin = 0;
  int gMax = 210;
  int gMin = 0;
  int bMax = 210;
  int bMin = 0;

  ConditionAnd<PointTT>::Ptr color_cond (new ConditionAnd<PointTT> ());
  color_cond->addComparison (PackedRGBComparison<PointTT>::Ptr (new PackedRGBComparison<PointTT> ("r", ComparisonOps::LT, rMax)));
  color_cond->addComparison (PackedRGBComparison<PointTT>::Ptr (new PackedRGBComparison<PointTT> ("r", ComparisonOps::GT, rMin)));
  color_cond->addComparison (PackedRGBComparison<PointTT>::Ptr (new PackedRGBComparison<PointTT> ("g", ComparisonOps::LT, gMax)));
  color_cond->addComparison (PackedRGBComparison<PointTT>::Ptr (new PackedRGBComparison<PointTT> ("g", ComparisonOps::GT, gMin)));
  color_cond->addComparison (PackedRGBComparison<PointTT>::Ptr (new PackedRGBComparison<PointTT> ("b", ComparisonOps::LT, bMax)));
  color_cond->addComparison (PackedRGBComparison<PointTT>::Ptr (new PackedRGBComparison<PointTT> ("b", ComparisonOps::GT, bMin)));

  // build the filter
  ConditionalRemoval<PointTT> condrem (color_cond);
  condrem.setInputCloud (cloud_in);
  condrem.setKeepOrganized(true);

  // apply filter
  condrem.filter (*cloud_in);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void passthrough(PointCloud<PointTT>::Ptr in, std::string field, float min, float max){
  PassThrough<PointTT> ps;
  ps.setInputCloud(in);
  ps.setFilterFieldName(field);
  ps.setFilterLimits(min, max);

  ps.filter(*in);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void remove_outlier(PointCloud<PointTT>::Ptr in, float mean, float deviation){
  StatisticalOutlierRemoval<PointTT> sor;
  sor.setInputCloud(in);
  sor.setMeanK(mean);
  sor.setStddevMulThresh(deviation);
  sor.filter(*in);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void filter_grid(PointCloud<PointTT>::Ptr in, float lf){
  VoxelGrid<PointTT> grid;
  grid.setInputCloud(in);
  grid.setLeafSize(lf, lf, lf);
  grid.filter(*in);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double bound180(double angle){
  if(angle >  180.0) angle = (angle - 360.0);
  if(angle < -180.0) angle = (angle + 360.0);

  return angle;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calculate_current_pose(Eigen::Quaternion<double> &q, Eigen::Vector3d &t, const sensor_msgs::PointCloud2ConstPtr msg_ptc){
  /////////////////////////// Frames ///////////////////////////
  ///      Camera                  Inercial
  ///
  ///      Z x--> X                   ^ U (Z)
  ///        |                        |
  ///      Y +                  N (Y) x--> E (X)
  ///
  /// YAW   na horizontal, positivo para direita, 0 no NORTE
  /// PITCH na vertical, positivo nariz pra baixo, 0 na HORIZONTAL
  /// ROLL  forcar a 0, a camera nao faz esse movimento normalmente

  // Diferenca da posicao norte e leste para o offset
  double dy = north_current - north_offset, dx = east_current - east_offset;
  // Diferenca do angulo de yaw para o offset - somente placa (HORARIO +, 0 NO NORTE)
  double dyaw = 0;//bound180( yaw_current_board - yaw_offset_board ); // [DEG]
  // Incremento do angulo de yaw segundo motor de PAN (HORARIO +, 0 para frente - offset)
  dyaw = DEG2RAD( bound180( dyaw + (pan_front - pan_current)*pwm2pan ) ); // [RAD]
  // Angulo de pitch segundo motor de TILT (positivo nariz pra baixo, 0 na horizontal - offset. Para isso, diferenca de angulos ao contrario)
  double dpitch = DEG2RAD( bound180( (tilt_current - tilt_hor)*pwm2tilt ) ); // [RAD]
  // Calculo do quaternion relativo - forcar roll a 0 (futuramente considerar leitura do viso2)
  q = AngleAxisd(dpitch, Vector3d::UnitX()) * AngleAxisd(dyaw, Vector3d::UnitY()) * AngleAxisd(0, Vector3d::UnitZ());
  // Calculo da translacao - rotacionar segundo angulo de offset de yaw
  double z_camera = dx*sin(DEG2RAD(yaw_offset_board)) + dy*cos(DEG2RAD(yaw_offset_board));
  double x_camera = dx*cos(DEG2RAD(yaw_offset_board)) - dy*sin(DEG2RAD(yaw_offset_board));
  t.data()[0] = 0;//x_camera;
  t.data()[1] =        0;
  t.data()[2] = 0;//z_camera;
  // Printar para averiguar
  if(true){
    cout << "\n###############################################################################"  << endl;
    cout << "Roll: " <<     0    << "\tPitch: " << RAD2DEG(dpitch) << "\tYaw: " << RAD2DEG(dyaw) << endl;
    cout << "X   : " << x_camera << "\tY    : " <<       0         << "\tZ  : " <<   z_camera    << endl;
    cout << "\n###############################################################################"  << endl;
  }
  // Publicar a odometria para a galera
  Odometry odom_out;
  odom_out.header.frame_id = msg_ptc->header.frame_id; odom_out.header.stamp = msg_ptc->header.stamp;
  odom_out.pose.pose.position.x = t.data()[0];
  odom_out.pose.pose.position.y = t.data()[1];
  odom_out.pose.pose.position.z = t.data()[2];
  odom_out.pose.pose.orientation.x = q.x(); odom_out.pose.pose.orientation.y = q.y();
  odom_out.pose.pose.orientation.z = q.z(); odom_out.pose.pose.orientation.w = q.w();
  pubodom.publish(odom_out);

}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void cloud_open_target2(const sensor_msgs::PointCloud2ConstPtr& msg_ptc,
                        const OdometryConstPtr& msg_odo,
                        const VFR_HUDConstPtr& msg_ang,
                        const OdometryConstPtr& msg_enu,
                        const OdometryConstPtr& msg_dyn){
  // Declare variables
  PointCloud<PointTT>::Ptr cloud (new PointCloud<PointTT>());
  PointCloud<PointTT>::Ptr cloud_transformed (new PointCloud<PointTT>());
  sensor_msgs::PointCloud2 msg_out;

  PointCloud<PointTT> cloud_acumulada_termica;
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
  passthrough(cloud, "z",   1, 20);
  passthrough(cloud, "x", -10, 10);
  passthrough(cloud, "y", -10, 10);
  // Filter for color
  filter_color(cloud);
  // Remove outiliers
  remove_outlier(cloud, 15, 0.5);

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
  if(false){
    Eigen::Matrix<double, 3, 1> euler;
    euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
    cout << "\n###############################################################################"  << endl;
    cout << "Roll: " << RAD2DEG(euler[0]) << "\tPitch: " << RAD2DEG(euler[1]) << "\tYaw: " << RAD2DEG(euler[2]) << endl;
    cout << "X   : " << offset(0)         << "\tY    : " << offset(1)         << "\tZ  : " << offset(2)         << endl;
    cout << "\n###############################################################################"  << endl;
  }

  /// Obter pose das mensagens
  if(first_read_mavros_dyn){
    yaw_offset_board  = DEG2RAD(msg_ang->groundspeed); // Vem em graus, devemos passar para radianos
    yaw_current_board = DEG2RAD(msg_ang->groundspeed);
    north_offset   = msg_enu->pose.pose.position.y; // [m]
    north_current  = msg_enu->pose.pose.position.y;
    north_previous = msg_enu->pose.pose.position.y;
    east_offset    = msg_enu->pose.pose.position.x;
    east_current   = msg_enu->pose.pose.position.x;
    east_previous  = msg_enu->pose.pose.position.x;
    pan_front     = msg_dyn->pose.pose.position.x; // O PAN  vem na mensagem na coordenada X do no $(find automatico_mrs)/controle_automatico
    pan_current   = msg_dyn->pose.pose.position.x;
    pan_previous  = msg_dyn->pose.pose.position.x;
    tilt_hor      = msg_dyn->pose.pose.position.y; // O TILT vem na mensagem na coordenada Y do no $(find automatico_mrs)/controle_automatico
    tilt_current  = msg_dyn->pose.pose.position.y;
    tilt_previous = msg_dyn->pose.pose.position.y;
//    vel_pan_previous  = msg_dyn->twist.twist.angular.x;
//    vel_pan_current   = msg_dyn->twist.twist.angular.x;
//    vel_tilt_previous = msg_dyn->twist.twist.angular.y;
//    vel_tilt_previous = msg_dyn->twist.twist.angular.y;
    first_read_mavros_dyn = false;
  } else {
    yaw_current_board = DEG2RAD(msg_ang->groundspeed);
    north_previous = north_current;
    east_previous  = east_current;
    north_current  = msg_enu->pose.pose.position.y;
    east_current   = msg_enu->pose.pose.position.x;
    pan_current  = msg_dyn->pose.pose.position.x;
    tilt_current = msg_dyn->pose.pose.position.y;
//    vel_pan_current   = msg_dyn->twist.twist.angular.x;
//    vel_tilt_current  = msg_dyn->twist.twist.angular.y;
  }
  // Calculo da pose a partir da placa, gps e dos motores
  Eigen::Quaternion<double> q2;
  Eigen::Vector3d offset2;
  calculate_current_pose(q2, offset2, msg_ptc);

  // Transformar a nuvem
//  transformPointCloud<PointTT>(*cloud, *cloud_transformed, offset, q);
  transformPointCloud<PointTT>(*cloud, *cloud_transformed, offset, q2);

  // Accumulate the point cloud using the += operator
//  ROS_INFO("Size of cloud_transformed = %ld", cloud_transformed->points.size());
  (*accumulated_cloud) += (*cloud_transformed);

//  ROS_INFO("Size of accumulated_cloud = %ld", accumulated_cloud->points.size());

//  // Separando as point clouds em visual e térmica
//  int nPontos = int(accumulated_cloud->points.size());
//  cloud_acumulada_termica.points.resize (nPontos);
//  for(int i = 0; i < int(accumulated_cloud->points.size()); i++)
//  {

//      cloud_acumulada_termica.points[i].x = accumulated_cloud->points[i].x;
//      cloud_acumulada_termica.points[i].y = accumulated_cloud->points[i].y;
//      cloud_acumulada_termica.points[i].z = accumulated_cloud->points[i].z;

//      cloud_acumulada_termica.points[i].r = accumulated_cloud->points[i].l;
//      cloud_acumulada_termica.points[i].g = accumulated_cloud->points[i].o;
//      cloud_acumulada_termica.points[i].b = accumulated_cloud->points[i].p;\

//      if(cloud_acumulada_termica.points[i].r != -1)
//      {
//          cloud_acumulada_termica.points[i].r = accumulated_cloud->points[i].l;
//          cloud_acumulada_termica.points[i].g = accumulated_cloud->points[i].o;
//          cloud_acumulada_termica.points[i].b = accumulated_cloud->points[i].p;\
//      }
//      else
//      {
//          cloud_acumulada_termica.points[i].x = nan("");
//          cloud_acumulada_termica.points[i].y = nan("");
//          cloud_acumulada_termica.points[i].z = nan("");
//          cloud_acumulada_termica.points[i].r = nan("");
//          cloud_acumulada_termica.points[i].g = nan("");
//          cloud_acumulada_termica.points[i].b = nan("");
//      }

//  }

//  // Limpar point cloud termica de elementos NAN
//  vector<int> indicesNAN2;
//  removeNaNFromPointCloud(cloud_acumulada_termica, cloud_acumulada_termica, indicesNAN2);

  // Convert the pcl point cloud to ros msg and publish
  toROSMsg(*accumulated_cloud, msg_out);
  msg_out.header.stamp = msg_ptc->header.stamp;
  pub->publish(msg_out);

  // Publicando point cloud térmica
  toROSMsg (cloud_acumulada_termica, termica_out);
  termica_out.header.stamp = msg_ptc->header.stamp;
  termica_out.header.frame_id = ros::names::remap("/odom");
  pub_termica->publish(termica_out);

  cloud.reset();
  cloud_transformed.reset();
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "accumulatepointcloud");
  ros::NodeHandle nh;

  // First time for all position data from mavros and dynamixel
  first_read_mavros_dyn = true;
  // Constantes de conversao dos valores de PWM para DEGREES de ambos os motores - dados findos de $(find automatico_mrs)/controle_automatico.cpp
  pwm2pan  = (300.0 -   0  ) / (1023.0 -    0  ); // [DEG/PWM]
  pwm2tilt = (205.0 - 153.0) / (2343.0 - 1746.0); // [DEG/PWM]

  // Initialize accumulated cloud variable
  accumulated_cloud = (PointCloud<PointTT>::Ptr) new PointCloud<PointTT>;
  accumulated_cloud->header.frame_id = ros::names::remap("/odom");

  // Initialize the point cloud publisher
  pub = (boost::shared_ptr<ros::Publisher>) new ros::Publisher;
  *pub = nh.advertise<sensor_msgs::PointCloud2>("/accumulated_point_cloud", 1);
  pub_termica = (boost::shared_ptr<ros::Publisher>) new ros::Publisher;
  *pub_termica = nh.advertise<sensor_msgs::PointCloud2>("/accumulated_termica", 1);

  // Subscriber para a nuvem instantanea e odometria
  message_filters::Subscriber<sensor_msgs::PointCloud2>  subptc(nh, "/stereo/points2"                 , 100);
  message_filters::Subscriber<Odometry>                  subodo(nh, "/stereo_odometer/odometry"       , 100);
  message_filters::Subscriber<VFR_HUD>                   subang(nh, "/mavros/vfr_hud"                 , 100);
  message_filters::Subscriber<Odometry>                  subenu(nh, "/mavros/global_position/local"   , 100);
//  message_filters::Subscriber<Odometry>                  subdyn(nh, "/dynamixel_sync"              , 100);
  message_filters::Subscriber<Odometry>                  subdyn(nh, "/dynamixel_angulos_sincronizados", 100);

  // Sincroniza as leituras dos topicos (sensores e imagem a principio) em um so callback
  Synchronizer<syncPolicy> sync(syncPolicy(100), subptc, subodo, subang, subenu, subdyn);
  sync.registerCallback(boost::bind(&cloud_open_target2, _1, _2, _3, _4, _5));

  // Publisher para a odometria calculada dos outros sensores
  pubodom = nh.advertise<Odometry>("/nossa_pose_agora", 100);

  // Loop infinitly
  ros::spin();

  return 0;

}
