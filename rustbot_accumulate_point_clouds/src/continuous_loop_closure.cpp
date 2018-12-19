//Includes
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_calibration_parsers/parse.h>
#include <camera_info_manager/camera_info_manager.h>
#include <yaml-cpp/yaml.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

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
using namespace geometry_msgs;
using namespace Eigen;
using namespace cv;

/// Definitions
typedef PointXYZRGB PointT;
typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2, Odometry> syncPolicy;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Variaveis globais
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Mat imagem_anterior;
Eigen::Quaternion<double> q_anterior, q_anterior_inverso, q_atual, q_relativo;
Eigen::Vector3d t_anterior, t_atual, t_relativo;
PointCloud<PointT>::Ptr cloud, cloud_tf, cloud_filt;
image_geometry::PinholeCameraModel model;

bool primeira_vez = true;

// Publicador
ros::Publisher cloud_pub;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Iniciar o modelo da camera
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void inicia_modelo_camera(sensor_msgs::CameraInfo ci){
  model.fromCameraInfo(ci);
//  cout << "Matriz intrinseca da camera:\n" << model.fullIntrinsicMatrix() << endl;
//  cout << "Matriz de projecao da camera:\n" << model.fullProjectionMatrix() << endl;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Retornar transformacao relativa entre os instantes
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calcula_transformacao_relativa(){
  q_anterior_inverso = q_anterior.inverse();
  q_relativo = q_anterior_inverso*q_atual;
  t_relativo = q_anterior_inverso*(t_atual - t_anterior);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Callback para evitar repeticao da nuvem por projecao na imagem anterior
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop_closure_callback(const sensor_msgs::ImageConstPtr& msg_ima,
                           const sensor_msgs::PointCloud2ConstPtr& msg_ptc,
                           const OdometryConstPtr& msg_odo){
  ROS_INFO("Entramos no callback de loop closure");

  // Ler nuvem de pontos
  fromROSMsg(*msg_ptc, *cloud);
  cloud_filt->header.frame_id = cloud->header.frame_id;
  cloud_filt->header.stamp    = cloud->header.stamp;

  // Remover NaN
  vector<int> indicesNAN;
  removeNaNFromPointCloud(*cloud, *cloud, indicesNAN);

  // Preparando mensagem da nuvem de saida
  sensor_msgs::PointCloud2 msg_ptc_out;
  msg_ptc_out.header.frame_id = msg_ptc->header.frame_id;
  msg_ptc_out.header.stamp    = msg_ptc->header.stamp;

  /// LOOP de PROJECAO sobre a nuvem
  if(!primeira_vez){
    // Pega rotacao e translacao atual
    q_atual.x() = (double)msg_odo->pose.pose.orientation.x;
    q_atual.y() = (double)msg_odo->pose.pose.orientation.y;
    q_atual.z() = (double)msg_odo->pose.pose.orientation.z;
    q_atual.w() = (double)msg_odo->pose.pose.orientation.w;
    t_atual = {msg_odo->pose.pose.position.x, msg_odo->pose.pose.position.y, msg_odo->pose.pose.position.z};

    // Calcular transformacao relativa entre os instantes
    calcula_transformacao_relativa();

    // Transformar camera para o ponto onde estava na iteracao anterior, e a nuvem para a iteracao de agora
    transformPointCloud<PointT>(*cloud, *cloud_tf, t_relativo, q_relativo);

    // Projeta cada ponto da nuvem na imagem e se cair fora adiciona na nuvem filtrada - OU REMOVE DA NUVEM
    cv::Point3d ponto3D;
    cv::Point2d pontoProjetado;
    for(int i=0; i < cloud_tf->size(); i++){
      ponto3D.x = cloud_tf->points[i].x;
      ponto3D.y = cloud_tf->points[i].y;
      ponto3D.z = cloud_tf->points[i].z;

      pontoProjetado = model.project3dToPixel(ponto3D);
      // Se cair na imagem da esquerda esta fora
      if(!(pontoProjetado.x > 0 && pontoProjetado.x < 2*imagem_anterior.cols-200 && pontoProjetado.y > 0 && pontoProjetado.y < 2*imagem_anterior.rows-100)){
        cloud_filt->push_back(cloud->points[i]); // Guardar o ponto sem transformacao relativa
      }
    }
    cout << "Tamanho antes da nuvem que nao foi projetada: " << cloud_filt->size() << endl;

    // Armazena para a proxima iteracao
    imagem_anterior = cv_bridge::toCvShare(msg_ima, "bgr8")->image;
    q_anterior = q_atual;
    t_anterior = t_atual;

    // Publica nuvem normalmente aqui
    toROSMsg(*cloud_filt, msg_ptc_out);
    cloud_pub.publish(msg_ptc_out);

  } else {

    // Armazena tudo nas variaveis anteriores
    imagem_anterior = cv_bridge::toCvShare(msg_ima, "bgr8")->image;

    q_anterior.x() = (double)msg_odo->pose.pose.orientation.x;
    q_anterior.y() = (double)msg_odo->pose.pose.orientation.y;
    q_anterior.z() = (double)msg_odo->pose.pose.orientation.z;
    q_anterior.w() = (double)msg_odo->pose.pose.orientation.w;
    t_anterior = {msg_odo->pose.pose.position.x, msg_odo->pose.pose.position.y, msg_odo->pose.pose.position.z};

    // Publica nuvem normalmente aqui
    toROSMsg(*cloud, msg_ptc_out);
    cloud_pub.publish(msg_ptc_out);

    primeira_vez = false;
  } // fim do if

  // Libera as nuvens
  cloud->clear(); cloud_filt->clear(); cloud_tf->clear();
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Main ///
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv){
  ros::init(argc, argv, "continuous_loop_closure");
  ros::NodeHandle nh;
  ros::NodeHandle n_("~"); // Frescura da camera

  // Aloca as nuvens
  cloud      = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;
  cloud_tf   = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;
  cloud_filt = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;

  // Inicia o publicador
  cloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("/loop_closure_cloud", 1000);

  // Inicia o modelo da camera com arquivo de calibracao
  std::string camera_calibration_yaml;
  n_.getParam("left_calibration_yaml", camera_calibration_yaml);
  camera_calibration_yaml = camera_calibration_yaml + std::string(".yaml");
  camera_info_manager::CameraInfoManager cam_info(n_, "left_optical", camera_calibration_yaml);
  inicia_modelo_camera(cam_info.getCameraInfo());

  // Subscriber para a nuvem instantanea e odometria
  message_filters::Subscriber<sensor_msgs::Image>       subima(nh, "/stereo/left/image_raw"   , 2000);
  message_filters::Subscriber<sensor_msgs::PointCloud2> subptc(nh, "/stereo/points2"          , 2000);
  message_filters::Subscriber<Odometry>                 subodo(nh, "/stereo_odometer/odometry", 2000);

  // Sincroniza as leituras dos topicos
  Synchronizer<syncPolicy> sync(syncPolicy(2000), subima, subptc, subodo);
  sync.registerCallback(boost::bind(&loop_closure_callback, _1, _2, _3));

  ros::Rate rate(1);

  while(ros::ok()){
    ros::spinOnce();
    rate.sleep();
  }
//  ros::spin();
}
