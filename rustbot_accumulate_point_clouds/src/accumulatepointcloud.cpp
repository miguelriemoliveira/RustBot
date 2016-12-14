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
pcl::VoxelGrid<PointT> grid;

tf::TransformListener *p_listener;

void cloud_open_target (const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_transformed(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg (*msg, *cloud);

    grid.setInputCloud (cloud);
    grid.filter (*cloud_filtered);

    pcl_ros::transformPointCloud(ros::names::remap("/map"), *cloud_filtered, *cloud_transformed, *p_listener);
    //ROS_INFO("cloud_transformed->points.size() = %ld", cloud_transformed->points.size());
    //cloud_transformed->header.frame_id = "/map";

    (*accumulated_cloud) += (*cloud_transformed);
    accumulated_cloud->header.frame_id;
    ROS_INFO("Size of accumulated_cloud = %ld", accumulated_cloud->points.size());
}


int main (int argc, char** argv)
{

    // Initialize ROS
    ros::init (argc, argv, "accumulatepointcloud2file");

    // Node Handle
    ros::NodeHandle nh;

    ros::param::get("~filename", filename);


    accumulated_cloud = (pcl::PointCloud<PointT>::Ptr) new pcl::PointCloud<PointT>;
    p_listener = (tf::TransformListener*) new tf::TransformListener;
    
    grid.setLeafSize (0.03f, 0.03f, 0.03f);

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub_target = nh.subscribe ("input", 1, cloud_open_target);


    while (ros::ok())
    {
        // Spin
        ros::spinOnce();
    }

    grid.setInputCloud (accumulated_cloud);
    grid.filter (*accumulated_cloud_filtered);

    printf("Saving to file %s\n", filename.c_str());;
    pcl::io::savePCDFileASCII (filename, *accumulated_cloud_filtered);

}
