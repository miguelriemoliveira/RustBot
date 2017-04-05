#ifndef SR_NODE_IMU_CORE_H
#define SR_NODE_IMU_CORE_H

// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"
#include  "std_msgs/Float64.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/MagneticField.h"
// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
#include <rustbot_bringup/imu_parameters.h>
#include "tf/transform_datatypes.h"

using std::string;

class IMU_replublish{
public:
    int n_msg;
    sensor_msgs::Imu IMU_data;
    ros::Publisher pub_message_IMU;
    double yaw_gps;
  //! Constructor.
  IMU_replublish();

  //! Destructor.
  ~IMU_replublish();

  //! Callback function for dynamic reconfigure server.
  void configCallback(rustbot_bringup::imu_parameters &config, uint32_t level);

  //! Publish the message.
  void publishMessage(ros::Publisher *pub_message );

  //! Callback function for subscriber.
  void IMU_data_messageCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void IMU_MAG_data_messageCallback(const  sensor_msgs::MagneticFieldConstPtr & msg);
  void IMU_data_yawmag_messageCallback(const std_msgs::Float64ConstPtr &msg);
  void IMU_yawfromGPSVelocity_messageCallback(const geometry_msgs::TwistStampedConstPtr &msg);

  //! The actual message.
  string message;

  //! The first integer to use in addition.
  int a;

  //! The second integer to use in addition.
  int b;
  int imu_deg;
};

#endif // SR_NODE_EXAMPLE_CORE_H
