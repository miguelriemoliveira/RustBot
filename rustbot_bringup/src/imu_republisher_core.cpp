#include "imu_republisher_core.h"

/*--------------------------------------------------------------------
 * IMU_replublish()
 * Constructor.
 *------------------------------------------------------------------*/

IMU_replublish::IMU_replublish()
{
    n_msg=0;
} // end IMU_replublish()

/*--------------------------------------------------------------------
 * ~IMU_replublish()
 * Destructor.
 *------------------------------------------------------------------*/

IMU_replublish::~IMU_replublish()
{
} // end ~IMU_replublish()

/*--------------------------------------------------------------------
 * publishMessage()
 * Publish the message.
 *------------------------------------------------------------------*/

void IMU_replublish::publishMessage(ros::Publisher *pub_message)
{
  //::node_example_data msg;
  //msg.message = message;
 /// msg.a = a;
  //msg.b = b;
    tf::Quaternion q;
    tf::quaternionMsgToTF(IMU_data.orientation,q);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll,pitch,yaw);
 std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;
  pub_message->publish(IMU_data);
} // end publishMessage()

/*--------------------------------------------------------------------
 * messageCallback()
 * Callback function for subscriber.
 *------------------------------------------------------------------*/

void IMU_replublish::IMU_data_messageCallback(const sensor_msgs::Imu::ConstPtr& msg){
    if(n_msg <20){
        n_msg++;
    }

    IMU_data = *msg;

    publishMessage(&pub_message_IMU);

  //message = msg->message;
 // a = msg->a;
  // b = msg->b;

  // Note that these are only set to INFO so they will print to a terminal for example purposes.
  // Typically, they should be DEBUG.
 // ROS_INFO("message is %s", message.c_str());
    // ROS_INFO("sum of a + b = %d", a + b);
}





void IMU_replublish::IMU_MAG_data_messageCallback(const sensor_msgs::MagneticFieldConstPtr &msg){

    std::cout << msg->magnetic_field.x << " | "<< msg->magnetic_field.y << " deu=" << atan2(msg->magnetic_field.y,msg->magnetic_field.x) << std::endl;



} // end publishCallback()

/*--------------------------------------------------------------------
 * configCallback()
 * Callback function for dynamic reconfigure server.
 *------------------------------------------------------------------*/

void IMU_replublish::configCallback(rustbot_bringup::imu_parameters &config, uint32_t level)
{
  // Set class variables to new values. They should match what is input at the dynamic reconfigure GUI.
  message = config.message.c_str();
  a = config.a;
  b = config.b;
} // end configCallback()

