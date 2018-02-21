//Includes
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <pcl/features/normal_3d.h> // add normals to render in MART.exe

//Definitions
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointNormal PS;

//Global vars
std::string filename  = "/tmp/output.pcd";
std::string filename2 = "/tmp/output.ply";
std::string filename3 = "/tmp/output_plus_normals.ply";
pcl::PointCloud<PointT>::Ptr accumulated_cloud;
tf::TransformListener *p_listener;
boost::shared_ptr<ros::Publisher> pub;

// Para gravar em PLY
pcl::PointCloud<PS> output;

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
    //pcl::removeNaNFromPointCloud(*accumulated_cloud, *accumulated_cloud, indicesNAN);

    //Get the transform, return if cannot get it
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


    //Transform point cloud using the transform obtained
    Eigen::Affine3d eigen_trf;                                  
    tf::transformTFToEigen (trans, eigen_trf);
    pcl::transformPointCloud<PointT>(*cloud, *cloud_transformed, eigen_trf);

    //Accumulate the point cloud using the += operator
    ROS_INFO("Size of cloud_transformed = %ld", cloud_transformed->points.size());
    (*accumulated_cloud) += (*cloud_transformed);

    //Voxel grid filter the accumulated cloud
    *tmp_cloud = *accumulated_cloud;
    grid.setInputCloud(tmp_cloud);
//    grid.setLeafSize (0.2f, 0.2f, 0.2f);
    grid.setLeafSize (0.05f, 0.05f, 0.05f);
    grid.filter (*accumulated_cloud);
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
  p_listener = (tf::TransformListener*) new tf::TransformListener;
  ros::Duration(2).sleep();

  //Initialize accumulated cloud variable
  accumulated_cloud = (pcl::PointCloud<PointT>::Ptr) new pcl::PointCloud<PointT>;
  accumulated_cloud->header.frame_id = ros::names::remap("/map");

  //Initialize the point cloud publisher
  pub = (boost::shared_ptr<ros::Publisher>) new ros::Publisher;
  *pub = nh.advertise<sensor_msgs::PointCloud2>("/accumulated_point_cloud", 1);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub_target = nh.subscribe ("input", 1, cloud_open_target);

  int contador = 0;

  //Loop infinitly
  while (ros::ok())
  {
    // Lets try to save it once in a while
    contador++;
    if(contador>600000){
      pcl::PointCloud<PointT> acc_cloud_test(*accumulated_cloud);
      if(acc_cloud_test.points.size() > 0){
        //Save accumulated point cloud to a file
//        printf("Saving to file %s\n", filename.c_str());
//        pcl::io::savePCDFileASCII (filename, *accumulated_cloud);

        // Lets try to compute normals
        //Initialize temp. clouds
        pcl::PointCloud<pcl::PointXYZ> temp_cloud1;
        pcl::PointCloud<PointT> temp_cloud2;
        temp_cloud2 = *accumulated_cloud;
        temp_cloud1.points.resize(temp_cloud2.size());
        for(int i=0; i < temp_cloud1.points.size(); i++){
          temp_cloud1.points[i].x = temp_cloud2.points[i].x;
          temp_cloud1.points[i].y = temp_cloud2.points[i].y;
          temp_cloud1.points[i].z = temp_cloud2.points[i].z;
        } // NOw just xyz to compute normals
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud1_ptr(&temp_cloud1);
        ne.setInputCloud (temp_cloud1_ptr);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
        ne.setSearchMethod (tree);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
        ne.setRadiusSearch (1);
        ne.compute(*cloud_normals);
        printf("Saving with normals to file %s\n", filename3.c_str());

        output.clear();
        pcl::PointCloud<pcl::Normal> temp_normal(*cloud_normals);
        output.points.resize(temp_cloud2.size());
        for(int i=0; i < output.points.size(); i++){
          output.points[i].x = temp_cloud2.points[i].x;
          output.points[i].y = temp_cloud2.points[i].y;
          output.points[i].z = temp_cloud2.points[i].z;
          output.points[i].normal_x = temp_normal.points[i].normal_x;
          output.points[i].normal_y = temp_normal.points[i].normal_y;
          output.points[i].normal_z = temp_normal.points[i].normal_z;
        }
        pcl::PointCloud<PS>::Ptr output_ptr(&output);
        pcl::io::savePLYFileASCII(filename3, *output_ptr);
        printf("All safe and sound!");

        temp_cloud1.clear();
        temp_cloud2.clear();
        temp_cloud1_ptr.reset();
        temp_normal.clear();
        output.clear();
        output_ptr.reset();
      }
    }
    // Spin
    ros::spinOnce();
  }

}


//      //Save accumulated point cloud to a file
//      printf("Saving to file %s\n", filename.c_str());
//      pcl::io::savePCDFileASCII (filename, *accumulated_cloud);
////      printf("Saving to file %s\n", filename2.c_str());
////      pcl::io::savePLYFileBinary(filename2, *accumulated_cloud);


//      // Lets try to compute normals
//      //Initialize temp. clouds
//      pcl::PointCloud<pcl::PointXYZ> temp_cloud1;
//      pcl::PointCloud<PointT> temp_cloud2;
//      temp_cloud2 = *accumulated_cloud;
//      temp_cloud1.points.resize(temp_cloud2.size());
//      for(int i=0; i < temp_cloud1.points.size(); i++){
//        temp_cloud1.points[i].x = temp_cloud2.points[i].x;
//        temp_cloud1.points[i].y = temp_cloud2.points[i].y;
//        temp_cloud1.points[i].z = temp_cloud2.points[i].z;
//      } // NOw just xyz to compute normals
//      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
//      pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud1_ptr(&temp_cloud1);
//      ne.setInputCloud (temp_cloud1_ptr);
//      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
//      ne.setSearchMethod (tree);
//      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
//      ne.setRadiusSearch (1);
//      ne.compute(*cloud_normals);
//      printf("Saving with normals to file %s\n", filename3.c_str());

//      pcl::PointCloud<pcl::PointXYZRGBNormal> output;
//      pcl::PointCloud<pcl::Normal> temp_normal(*cloud_normals);
//      output.points.resize(temp_cloud2.size());
//      for(int i=0; i < output.points.size(); i++){
//        output.points[i].x = temp_cloud2.points[i].x;
//        output.points[i].y = temp_cloud2.points[i].y;
//        output.points[i].z = temp_cloud2.points[i].z;
//        output.points[i].r = temp_cloud2.points[i].r;
//        output.points[i].g = temp_cloud2.points[i].g;
//        output.points[i].b = temp_cloud2.points[i].b;
//        output.points[i].normal_x = temp_normal.points[i].normal_x;
//        output.points[i].normal_y = temp_normal.points[i].normal_y;
//        output.points[i].normal_z = temp_normal.points[i].normal_z;
//      }
//      pcl::io::savePLYFileASCII(filename3, output);
//      printf("All safe and sound!");
