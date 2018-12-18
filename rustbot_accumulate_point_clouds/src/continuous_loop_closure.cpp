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
typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2, Odometry> syncPolicy;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Variaveis globais
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Mat imagem_anterior;
Odometry odom_anterior;
sensor_msgs::CameraInfo cam_info;
Eigen::Quaternion<double> q_anterior, q_atual;
Eigen::Vector3d offset_anterior, offset_atual;
PointCloud<PointT>::Ptr cloud;
image_geometry::PinholeCameraModel model;
Mat T_camera, P_camera, R, t;

bool primeira_vez = true;

// Publicador
ros::Publisher cloud_pub;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Callback para evitar repeticao da nuvem por projecao na imagem anterior
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop_closure_callback(const sensor_msgs::ImageConstPtr& msg_ima,
                           const sensor_msgs::CameraInfoConstPtr& msg_cam,
                           const sensor_msgs::PointCloud2ConstPtr& msg_ptc,
                           const OdometryConstPtr& msg_odo){
  ROS_INFO("Entramos no callback de loop closure");
  // Ler nuvem de pontos
  fromROSMsg(*msg_ptc, *cloud);

  // Ler camera info
//  imagem_atual = cv_bridge::toCvShare(msg_ima, "bgr8")->image;
//  odom_atual = *msg_odo;
  cam_info = *msg_cam;

  // Remover NaN
  vector<int> indicesNAN;
  removeNaNFromPointCloud(*cloud, *cloud, indicesNAN);

  // Preparando mensagem da nuvem de saida
  sensor_msgs::PointCloud2 msg_ptc_out;
  msg_ptc_out.header.frame_id = cloud->header.frame_id;
//  msg_ptc_out.header.stamp    = cloud->header.stamp;

  /// LOOP de PROJECAO sobre a nuvem
  if(!primeira_vez){
    // Pega rotacao e translacao atual
    q_atual.x() = (double)msg_odo->pose.pose.orientation.x;
    q_atual.y() = (double)msg_odo->pose.pose.orientation.y;
    q_atual.z() = (double)msg_odo->pose.pose.orientation.z;
    q_atual.w() = (double)msg_odo->pose.pose.orientation.w;
    offset_atual = {msg_odo->pose.pose.position.x, msg_odo->pose.pose.position.y, msg_odo->pose.pose.position.z};

    // Transformar camera para o ponto onde estava na iteracao anterior, e a nuvem para a iteracao de agora
    transformPointCloud<PointT>(*cloud, *cloud, offset_atual, q_atual);
    cout << "Parametros intrinsecos da camera:\n" << model.fullIntrinsicMatrix() << endl;
//    R = Mat(3, 3, CV_64F, q_anterior.matrix());
    R = Mat::eye(3, 3, CV_64F);
    cout << "Matriz vinda do quaternion:\n" << q_anterior.matrix() << endl;
    T_camera.push_back(R);
    t = (cv::Mat1d(3, 1, CV_64F) << offset_anterior.data()[0], offset_anterior.data()[1], offset_anterior.data()[2]);
    T_camera.push_back(t);
    cout << "Matriz T final:\n" << T_camera << endl;
//    cam_info.P = cam_info.K*T_camera;

    // Com a matriz de projecao correta ajustar o modelo da camera
    model.fromCameraInfo(cam_info);

    // Projeta cada ponto da nuvem na imagem e se cair fora adiciona na nuvem filtrada - OU REMOVE DA NUVEM
    cv::Point3d ponto3D;
    cv::Point2d pontoProjetado;
    for(PointCloud<PointT>::iterator it = cloud->begin(); it!=cloud->end(); it++){
      ponto3D.x = it->x;
      ponto3D.y = it->y;
      ponto3D.z = it->z;

      pontoProjetado = model.project3dToPixel(ponto3D);
      // Se dentro da imagem, apaga da nuvem - mais rapido
      if(pontoProjetado.x > 0  && pontoProjetado.x < imagem_anterior.cols && pontoProjetado.y > 0 && pontoProjetado.y < imagem_anterior.rows){
        cloud->erase(it);
        it++;
      }
    }

    // Armazena para a proxima iteracao
    imagem_anterior = cv_bridge::toCvShare(msg_ima, "bgr8")->image;
    odom_anterior = *msg_odo;
    q_anterior = q_atual;
    offset_anterior = offset_atual;

    // Publica nuvem normalmente aqui
    toROSMsg(*cloud, msg_ptc_out);
    cloud_pub.publish(msg_ptc_out);

  } else {

    // Armazena tudo nas variaveis anteriores
    imagem_anterior = cv_bridge::toCvShare(msg_ima, "bgr8")->image;
    odom_anterior = *msg_odo;

    q_anterior.x() = (double)msg_odo->pose.pose.orientation.x;
    q_anterior.y() = (double)msg_odo->pose.pose.orientation.y;
    q_anterior.z() = (double)msg_odo->pose.pose.orientation.z;
    q_anterior.w() = (double)msg_odo->pose.pose.orientation.w;
    offset_anterior = {msg_odo->pose.pose.position.x, msg_odo->pose.pose.position.y, msg_odo->pose.pose.position.z};

    // Publica nuvem normalmente aqui
    toROSMsg(*cloud, msg_ptc_out);
    cloud_pub.publish(msg_ptc_out);

    primeira_vez = false;
  } // fim do if

  // Libera as nuvens e matrizes
  cloud->clear();
  R.release(); t.release(); T_camera.release(); P_camera.release();
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Main ///
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv){
  ros::init(argc, argv, "continuous_loop_closure");
  ros::NodeHandle nh;

  // Aloca as nuvens
  cloud      = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;

  // Inicia o publicador
  cloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("/loop_closure_cloud", 1000);

  // Subscriber para a nuvem instantanea e odometria
  message_filters::Subscriber<sensor_msgs::Image>       subima(nh, "/stereo/left/image_rect"  , 200);
  message_filters::Subscriber<sensor_msgs::CameraInfo>  subcam(nh, "/stereo/left/camera_info" , 200);
  message_filters::Subscriber<sensor_msgs::PointCloud2> subptc(nh, "/stereo/points2"          , 200);
  message_filters::Subscriber<Odometry>                 subodo(nh, "/stereo_odometer/odometry", 200);

  // Sincroniza as leituras dos topicos
  Synchronizer<syncPolicy> sync(syncPolicy(200), subima, subcam, subptc, subodo);
  sync.registerCallback(boost::bind(&loop_closure_callback, _1, _2, _3, _4));

  ros::spin();
}
