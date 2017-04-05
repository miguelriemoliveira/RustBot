#include "imu_republisher_core.h"
#include "mavros_msgs/StreamRate.h"

/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/

int main(int argc, char **argv){
  // Set up ROS.
  ros::init(argc, argv, "IMU_republisher");
  ros::NodeHandle n;

  // Declare variables that can be modified by launch file or command line.
  int rate;
  bool simulator;
  string topic;

  sleep(5);
  // Create a new IMU_replublish object.
  IMU_replublish *IMU_node_handler = new IMU_replublish();

  // Initialize node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can be run simultaneously
  // while using different parameters.
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("rate", rate, int(40));
  private_node_handle_.param("not_sim", simulator , bool(true));
  private_node_handle_.param("topic", topic, string("/mavros/imu/data"));

  // Create a subscriber.
  // Name the topic, message queue, callback function with class name, and object containing callback function.
  ros::Subscriber sub_message = n.subscribe(topic.c_str(), 1000, &IMU_replublish::IMU_data_messageCallback, IMU_node_handler);
  ros::Subscriber sub_message1 = n.subscribe("/mavros/imu/mag", 1000, &IMU_replublish::IMU_MAG_data_messageCallback, IMU_node_handler);
  ros::Subscriber sub_message2 = n.subscribe("/mavros/global_position/compass_hdg", 1000,&IMU_replublish::IMU_data_yawmag_messageCallback, IMU_node_handler);
  ros::Subscriber sub_message3 = n.subscribe("/mavros/global_position/raw/gps_vel", 1000,&IMU_replublish::IMU_yawfromGPSVelocity_messageCallback, IMU_node_handler);

  IMU_node_handler->pub_message_IMU = n.advertise<sensor_msgs::Imu>("imu_data_ENU", 40);

  // Tell ROS how fast to run this node.
  ros::Rate r(rate);
  ros::ServiceClient client = n.serviceClient<mavros_msgs::StreamRateRequest>("/mavros/set_stream_rate");
  mavros_msgs::StreamRate srv;
  srv.request.message_rate=20;
  srv.request.stream_id=0;
  srv.request.on_off=1;

  // if the system is not running in simulation the service mavros is not used
  if(simulator){
    ros::service::waitForService("/mavros/set_stream_rate", -1);
    ROS_INFO("MAVROS SERVICE is available..");
    if (client.call(srv)){
        ROS_INFO("Configure the MAVROS RATE");
        std::cout << topic.c_str() << std::endl;
    }else{
        ROS_ERROR("Failed to call service /mavros/set_stream_rate");
        return 1;
      }
  }


   // Main loop.
  int n_call=0;
  while (n.ok()){
    ros::spinOnce();
    r.sleep();
    if(IMU_node_handler->n_msg < 20){
        n_call++;
        if(n_call > 20){
            n_call=0;
            if(simulator){
                if (client.call(srv)){
                    ROS_INFO("Configure the MAVROS RATE");
                }else{
                    ROS_ERROR("Failed to call service /mavros/set_stream_rate");
                }
            }
        }
    }

  }

  return 0;
} // end main()
