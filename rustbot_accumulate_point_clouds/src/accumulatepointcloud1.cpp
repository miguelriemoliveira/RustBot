//Includes
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>

//Definitions
typedef pcl::PointXYZRGB PointT;

//Global vars
pcl::PointCloud<PointT>::Ptr accumulated_cloud;
tf::TransformListener *tf_odo_map;
tf::TransformListener *tf_baselink_odo;
boost::shared_ptr<ros::Publisher> pub;

void cloud_open_target(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    //declare variables
    pcl::PointCloud<PointT>::Ptr cloud;
    pcl::PointCloud<PointT>::Ptr cloud_transformed;
    pcl::PointCloud<PointT>::Ptr tmp_cloud;
    pcl::VoxelGrid<PointT> grid;
    sensor_msgs::PointCloud2 msg_out;

    //allocate objects for pointers
    cloud = (pcl::PointCloud<PointT>::Ptr) (new pcl::PointCloud<PointT>);
    cloud_transformed = (pcl::PointCloud<PointT>::Ptr) (new pcl::PointCloud<PointT>);
    tmp_cloud = (pcl::PointCloud<PointT>::Ptr) (new pcl::PointCloud<PointT>);


    //Convert the ros message to pcl point cloud
    pcl::fromROSMsg (*msg, *cloud);

    //Remove any NAN points in the cloud
    std::vector<int> indicesNAN;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indicesNAN);

    grid.setInputCloud(cloud);
    grid.setLeafSize (0.05f, 0.05f, 0.05f);
    grid.filter (*cloud);

    //Get the transform, return if cannot get it
    ros::Time tic = ros::Time::now();
    ros::Time t = msg->header.stamp;
    tf::StampedTransform trans1, trans2;
    try
    {
        tf_odo_map->waitForTransform("map", "odom", t, ros::Duration(3.0));
        tf_odo_map->lookupTransform("map", "odom", t, trans1);
        tf_baselink_odo->waitForTransform("odom", "base_link", t, ros::Duration(3.0));
        tf_baselink_odo->lookupTransform("odom", "base_link", t, trans2);
    }
    catch (tf::TransformException& ex){
        ROS_ERROR("%s",ex.what());
        //ros::Duration(1.0).sleep();
        ROS_WARN("Cannot accumulate");
        return;
    }
    ROS_INFO("Collected transforms (%0.3f secs)", (ros::Time::now() - tic).toSec());


    //Transform point cloud using the transform obtained
    Eigen::Affine3d eigen_trf1, eigen_trf2;
    tf::transformTFToEigen (trans2, eigen_trf2);
    pcl::transformPointCloud<PointT>(*cloud, *cloud_transformed, eigen_trf2);
    tf::transformTFToEigen (trans1, eigen_trf1);
    pcl::transformPointCloud<PointT>(*cloud_transformed, *cloud_transformed, eigen_trf1);

    //Accumulate the point cloud using the += operator
    ROS_INFO("Size of cloud_transformed = %ld", cloud_transformed->points.size());
    (*accumulated_cloud) += (*cloud_transformed);

    ROS_INFO("Size of accumulated_cloud = %ld", accumulated_cloud->points.size());

    //Conver the pcl point cloud to ros msg and publish
    pcl::toROSMsg (*accumulated_cloud, msg_out);
    msg_out.header.stamp = t;
    pub->publish(msg_out);

    cloud.reset();
    tmp_cloud.reset();
    cloud_transformed.reset();
}


int main (int argc, char** argv)
{

    // Initialize ROS
    ros::init (argc, argv, "accumulatepointcloud");
    ros::NodeHandle nh;

    //initialize the transform listenerand wait a bit
    tf_baselink_odo = (tf::TransformListener*) new tf::TransformListener;
    tf_odo_map      = (tf::TransformListener*) new tf::TransformListener;
    ros::Duration(2).sleep();

    //Initialize accumulated cloud variable
    accumulated_cloud = (pcl::PointCloud<PointT>::Ptr) new pcl::PointCloud<PointT>;
    accumulated_cloud->header.frame_id = "map";

    //Initialize the point cloud publisher
    pub = (boost::shared_ptr<ros::Publisher>) new ros::Publisher;
    *pub = nh.advertise<sensor_msgs::PointCloud2>("/accumulated_point_cloud", 1);

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub_target = nh.subscribe ("/stereo/points2", 1, cloud_open_target);

    //Loop infinetly
    while (ros::ok())
    {
        // Spin
        ros::spinOnce();
    }
}
