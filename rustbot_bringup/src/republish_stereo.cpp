#include <ros/ros.h>
#include <cmath>
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
using namespace std;
using namespace message_filters;
using namespace cv;

/// Definitions
typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo> syncPolicy;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Variaveis globais
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ros::Publisher esqpub;
ros::Publisher dirpub;
ros::Publisher cesqpub;
ros::Publisher cdirpub;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Callback
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void callback(const sensor_msgs::ImageConstPtr& msg_esq,
              const sensor_msgs::CameraInfoConstPtr& msg_ce,
              const sensor_msgs::ImageConstPtr& msg_dir,
              const sensor_msgs::CameraInfoConstPtr& msg_cd){
    // Convertendo as imagens
    cv_bridge::CvImagePtr esq = cv_bridge::toCvCopy(msg_esq, sensor_msgs::image_encodings::BGR8);
    // Alterando a imagem esquerda
    cv::Mat temp;
    esq->image.copyTo(temp);
    #pragma omp parallel for num_threads(60)
    for(int y=0; y < esq->image.rows; y++){
        for(int x=0; x < esq->image.cols; x++){
            Vec3b color = temp.at<Vec3b>(Point(x, y));
            esq->image.at<Vec3b>(Point(x, y)) = color;
        }
    }
    // Publicando com mesmo timestamp os resultados
    sensor_msgs::Image msg_esq2, msg_dir2;
    sensor_msgs::CameraInfo msg_ce2, msg_cd2;
    msg_esq2 = *esq->toImageMsg();
    msg_dir2 = *msg_dir;
    msg_esq2.header.stamp = ros::Time::now();//msg_esq->header.stamp;
    msg_dir2.header.stamp = msg_esq2.header.stamp;

    msg_ce2 = *msg_ce; msg_cd2 = *msg_cd;
    msg_ce2.header.stamp = msg_esq2.header.stamp;
    msg_cd2.header.stamp = msg_esq2.header.stamp;

    esqpub.publish(msg_esq2);
    dirpub.publish(msg_dir2);
    cesqpub.publish(msg_ce2);
    cdirpub.publish(msg_cd2);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Main
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ros::init(argc, argv, "republish_stereo");
  ros::NodeHandle nh;

  // Publishers sincronizados
  esqpub  = nh.advertise<sensor_msgs::Image     >("/stereo/left/image_raw2"   , 1000);
  cesqpub = nh.advertise<sensor_msgs::CameraInfo>("/stereo/left/camera_info2" , 1000);
  dirpub  = nh.advertise<sensor_msgs::Image     >("/stereo/right/image_raw2"  , 1000);
  cdirpub = nh.advertise<sensor_msgs::CameraInfo>("/stereo/right/camera_info2", 1000);

  // Subscriber para as imagens
  message_filters::Subscriber<sensor_msgs::Image     > subesq( nh, "/stereo/left/image_raw"   , 100);
  message_filters::Subscriber<sensor_msgs::CameraInfo> subcesq(nh, "/stereo/left/camera_info" , 100);
  message_filters::Subscriber<sensor_msgs::Image     > subdir( nh, "/stereo/left/image_raw"   , 100);
  message_filters::Subscriber<sensor_msgs::CameraInfo> subcdir(nh, "/stereo/right/camera_info", 100);

  // Sincroniza as leituras dos topicos
  Synchronizer<syncPolicy> sync(syncPolicy(100), subesq, subcesq, subdir, subcdir);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

  // Spin
  ros::spin();
}
