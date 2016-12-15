#include <ros/ros.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/transforms.h>

typedef pcl::PointXYZRGB PointT;

std::string filename = "output.pcd";

pcl::PointCloud<PointT>::Ptr accumulated_cloud;
pcl::PointCloud<PointT>::Ptr accumulated_cloud_filtered;

pcl::PointCloud<PointT>::Ptr cloud;
pcl::PointCloud<PointT>::Ptr tmp_cloud;
pcl::PointCloud<PointT>::Ptr cloud_transformed;

sensor_msgs::PointCloud2 msg_out;

pcl::VoxelGrid<PointT> grid;

tf::TransformListener *p_listener;
boost::shared_ptr<ros::Publisher> pub;

void cloud_open_target (const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::fromROSMsg (*msg, *cloud);

    std::vector<int> indicesNAN;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indicesNAN);

    ros::Time tic = ros::Time::now();
    ros::Time t = msg->header.stamp;
    tf::StampedTransform trans;
    try
    {
        p_listener->waitForTransform(ros::names::remap("/map"), msg->header.frame_id, t, ros::Duration(3.0));
        p_listener->lookupTransform(ros::names::remap("/map"), msg->header.frame_id, t, trans);
    }
    catch (tf::TransformException& ex){
        ROS_ERROR("%s",ex.what());
        //ros::Duration(1.0).sleep();
        ROS_WARN("Cannot accumulate");
        return;
    }
    ROS_INFO("Collected transforms (%0.3f secs)", (ros::Time::now() - tic).toSec());


    //try{
        //p_listener->waitForTransform(msg->header.frame_id, ros::names::remap("/map"), msg->header.stamp, ros::Duration(2.0));
        //pcl_ros::transformPointCloud(ros::names::remap("/map"), *cloud, *cloud_transformed, *p_listener);
    //}
    //catch (tf::TransformException ex)
    //{
        //ROS_ERROR("%s",ex.what());
        //return;
    //}


    //Transform point cloud
    Eigen::Affine3d eigen_trf;                                  
    tf::transformTFToEigen (trans, eigen_trf);
    pcl::transformPointCloud<PointT>(*cloud, *cloud_transformed, eigen_trf);



    (*accumulated_cloud) += (*cloud_transformed);

    grid.setInputCloud (accumulated_cloud);
    grid.setLeafSize (0.03f, 0.03f, 0.03f);
    grid.filter (*accumulated_cloud);
    ROS_INFO("Size of accumulated_cloud = %ld", accumulated_cloud->points.size());

    pcl::toROSMsg (*accumulated_cloud, msg_out);
    msg_out.header.stamp = ros::Time::now();
    pub->publish(msg_out);
}


int main (int argc, char** argv)
{

    // Initialize ROS
    ros::init (argc, argv, "accumulatepointcloud");

    // Node Handle
    ros::NodeHandle nh;

    accumulated_cloud = (pcl::PointCloud<PointT>::Ptr) new pcl::PointCloud<PointT>;
    cloud = (pcl::PointCloud<PointT>::Ptr) (new pcl::PointCloud<PointT>);
    cloud_transformed = (pcl::PointCloud<PointT>::Ptr) (new pcl::PointCloud<PointT>);
    tmp_cloud = (pcl::PointCloud<PointT>::Ptr) (new pcl::PointCloud<PointT>);



    accumulated_cloud->header.frame_id = ros::names::remap("/map");
    p_listener = (tf::TransformListener*) new tf::TransformListener;


    ros::Duration(2).sleep();

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub_target = nh.subscribe ("input", 1, cloud_open_target);

    pub = (boost::shared_ptr<ros::Publisher>) new ros::Publisher;
    *pub = nh.advertise<sensor_msgs::PointCloud2>("/accumulated_point_cloud", 1);


    while (ros::ok())
    {
        // Spin
        ros::spinOnce();
    }


    //printf("Saving to file %s\n", filename.c_str());;
    //pcl::io::savePCDFileASCII (filename, *accumulated_cloud);
}
