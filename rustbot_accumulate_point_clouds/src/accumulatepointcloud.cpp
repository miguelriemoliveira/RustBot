#include <ros/ros.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/transforms.h>

typedef pcl::PointXYZRGBA PointT;

std::string filename = "output.pcd";

pcl::PointCloud<PointT>::Ptr accumulated_cloud;
pcl::PointCloud<PointT>::Ptr accumulated_cloud_filtered;

pcl::PointCloud<PointT>::Ptr cloud;
pcl::PointCloud<PointT>::Ptr tmp_cloud;
pcl::PointCloud<PointT>::Ptr cloud_transformed;

pcl::VoxelGrid<PointT> grid;

tf::TransformListener *p_listener;
boost::shared_ptr<ros::Publisher> pub;

void cloud_open_target (const sensor_msgs::PointCloud2ConstPtr& msg)
{

    pcl::fromROSMsg (*msg, *cloud);

    ROS_INFO("Size of cloud = %ld", cloud->points.size());

    printf("LINE=%d\n",__LINE__);
    pcl_ros::transformPointCloud(ros::names::remap("/map"), *cloud, *cloud_transformed, *p_listener);
    printf("LINE=%d\n",__LINE__);


    ROS_INFO("Size of input cloud = %ld", cloud_transformed->points.size());
    printf("LINE=%d\n",__LINE__);
    (*accumulated_cloud) += (*cloud_transformed);
    printf("LINE=%d\n",__LINE__);


    ROS_INFO("Size of accumulated_cloud = %ld", accumulated_cloud->points.size());
    (*tmp_cloud) = *accumulated_cloud;
    printf("LINE=%d\n",__LINE__);
    grid.setInputCloud (tmp_cloud);
    printf("LINE=%d\n",__LINE__);
    grid.filter (*accumulated_cloud);
    printf("LINE=%d\n",__LINE__);

    ROS_INFO("Size of accumulated_cloud = %ld", accumulated_cloud->points.size());

    
    sensor_msgs::PointCloud2 msg_out;
    printf("LINE=%d\n",__LINE__);
    pcl::toROSMsg (*cloud, msg_out);
    //pcl::toROSMsg (*accumulated_cloud, msg_out);
    printf("LINE=%d\n",__LINE__);
    //msg_out.header.stamp = ros::Time::now();
    //printf("LINE=%d\n",__LINE__);
    //msg_out.is_dense = 0;
    printf("LINE=%d\n",__LINE__);
    pub->publish(msg_out);
    printf("LINE=%d\n",__LINE__);

    ROS_INFO("Size of msg_out = %d", msg_out.width);
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
    
    grid.setLeafSize (0.1f, 0.1f, 0.1f);
    //grid.setInputCloud (accumulated_cloud);
    //grid.filter (*accumulated_cloud_filtered);

    ros::Duration(2).sleep();

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub_target = nh.subscribe ("input", 1, cloud_open_target);

    pub = (boost::shared_ptr<ros::Publisher>) new ros::Publisher;
    *pub = nh.advertise<sensor_msgs::PointCloud2>("accumulated_point_cloud", 1);


    while (ros::ok())
    {
        // Spin
        ros::spinOnce();
    }

}
