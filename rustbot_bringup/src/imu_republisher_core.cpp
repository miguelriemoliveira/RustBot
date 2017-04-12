#include "imu_republisher_core.h"

/*--------------------------------------------------------------------
 * IMU_replublish()
 * Constructor.
 *------------------------------------------------------------------*/

IMU_replublish::IMU_replublish(){
    n_msg=0;
    imu_deg=0;
} // end IMU_replublish()

/*--------------------------------------------------------------------
 * ~IMU_replublish()
 * Destructor.
 *------------------------------------------------------------------*/

IMU_replublish::~IMU_replublish(){
} // end ~IMU_replublish()

/*--------------------------------------------------------------------
 * publishMessage()
 * Publish the message.
 *------------------------------------------------------------------*/

void IMU_replublish::publishMessage(ros::Publisher *pub_message){
  //::node_example_data msg;
  //msg.message = message;
    pub_message->publish(IMU_data);
}

/*--------------------------------------------------------------------
 * messageCallback()
 * Callback function for subscriber.
 *------------------------------------------------------------------*/
void IMU_replublish::IMU_data_messageCallback(const sensor_msgs::Imu::ConstPtr& msg){
    if(n_msg <20){
        n_msg++;
    }
  // Note that these are only set to INFO so they will print to a terminal for example purposes.
  // Typically, they should be DEBUG.
  // ROS_INFO("message is %s", message.c_str());
  // ROS_INFO("sum of a + b = %d", a + b);
}
/*--------------------------------------------------------------------
 * messageCallback()
 * Callback function for subscriber.
 *------------------------------------------------------------------*/
void IMU_replublish::IMU_MAG_data_messageCallback(const sensor_msgs::MagneticFieldConstPtr &msg){
  //  std::cout << msg->magnetic_field.x << " | "<< msg->magnetic_field.y << " deu=" << atan2(msg->magnetic_field.y,msg->magnetic_field.x) << std::endl;
    tf::Matrix3x3 obs_mat;
    obs_mat.setRPY(0,0, yaw_gps ); //atan2(msg->magnetic_field.y, msg->magnetic_field.x));
    tf::Quaternion qt_tf;
    obs_mat.getRotation(qt_tf);

    //std::cout << "IMU" << (-atan2(msg->magnetic_field.y, msg->magnetic_field.x)+1.57)* 180.0/ 3.14 << std::endl;
    IMU_data.orientation.x=qt_tf.getX();
    IMU_data.orientation.y=qt_tf.getY();
    IMU_data.orientation.z=qt_tf.getZ();
    IMU_data.orientation.w=qt_tf.getW();
    IMU_data.orientation_covariance.at(0)=0.01;
    IMU_data.orientation_covariance.at(4)=0.01;
    IMU_data.orientation_covariance.at(8)=0.1;
    IMU_data.header.frame_id.assign("imu_frame");
    publishMessage(&pub_message_IMU);
    imu_deg=1;
}

/*--------------------------------------------------------------------
 * messageCallback()
 * Callback function for subscriber.
 *------------------------------------------------------------------*/
void IMU_replublish::IMU_yawfromGPSVelocity_messageCallback(const geometry_msgs::TwistStampedConstPtr &msg){

    // std::cout << "GPS "<< msg->twist.linear.x << " | "<< msg->twist.linear.y << " deu=" << atan2(msg->twist.linear.x,msg->twist.linear.y) * 180.0/3.14  << std::endl;

     //
    // ENU ---> NED
    // roll ---->roll
    // pitch ----->-pitch
    // yaw ----->-yaw+1.57.
     yaw_gps=-atan2(msg->twist.linear.x,msg->twist.linear.y)+1.57;
     // std::cout << "GPS corrigido "<< yaw_gps * 180.0/3.14  << std::endl;
}

/*--------------------------------------------------------------------
 * messageCallback()
 * Callback function for subscriber.
 *------------------------------------------------------------------*/
void IMU_replublish::IMU_data_yawmag_messageCallback(const std_msgs::Float64ConstPtr &msg){
  // std::cout << " deu=" << -msg->data+1.57*180/3.14  << std::endl;

    if( imu_deg==1 ) return;

  tf::Matrix3x3 obs_mat;
  obs_mat.setRPY(0,0,msg->data * 3.14/180.0);
  tf::Quaternion qt_tf;
  obs_mat.getRotation(qt_tf);

  IMU_data.orientation.x=qt_tf.getX();
  IMU_data.orientation.y=qt_tf.getY();
  IMU_data.orientation.z=qt_tf.getZ();
  IMU_data.orientation.w=qt_tf.getW();
  IMU_data.orientation_covariance.at(0)=1.0;
  IMU_data.orientation_covariance.at(4)=1.0;
  IMU_data.orientation_covariance.at(8)=1.0;
  IMU_data.header.frame_id.assign("imu_frame");
  publishMessage(&pub_message_IMU);
   //std::cout << " ============" <<std::endl;

} // end publishCallback()

/*--------------------------------------------------------------------
 * configCallback()
 * Callback function for dynamic reconfigure server.
 *------------------------------------------------------------------*/
void IMU_replublish::configCallback(rustbot_bringup::imu_parameters &config, uint32_t level){
  // Set class variables to new values. They should match what is input at the dynamic reconfigure GUI.
  message = config.message.c_str();
  a = config.a;
  b = config.b;
} // end configCallback()

