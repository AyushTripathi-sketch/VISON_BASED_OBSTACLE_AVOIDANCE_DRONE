#include <ros/ros.h> 
#include <pcl/point_cloud.h> 
#include <pcl_conversions/pcl_conversions.h> 
#include <sensor_msgs/PointCloud2.h> 
#include <pcl/filters/passthrough.h>
//Vortex filter header 
#include <pcl/filters/voxel_grid.h> 
 
//Creating a class for handling cloud data 
class cloudHandler 
{ 
public: 
    cloudHandler() 
    { 
         
//Subscribing pcl_output topics from the publisher 
//This topic can change according to the source of point cloud 
 
        pcl_sub = nh.subscribe("/depth_camera/depth/points", 10, &cloudHandler::cloudCB, this); 
//Creating publisher for filtered cloud data 
        pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_filtered", 1); 
    } 
//Creating cloud callback 
    void cloudCB(const sensor_msgs::PointCloud2& input) 
    { 
        pcl::PointCloud<pcl::PointXYZ> cloud; 
        pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
        pcl::PointCloud<pcl::PointXYZ> cloud_pass_throughed; 
 
      
       sensor_msgs::PointCloud2 output; 
       pcl::fromROSMsg(input, cloud); 
 
     //Creating VoxelGrid object 
      pcl::VoxelGrid<pcl::PointXYZ> vox_obj; 
     //Set input to voxel object 
     vox_obj.setInputCloud (cloud.makeShared()); 
   
     //Setting parameters of filter such as leaf size 
    vox_obj.setLeafSize (0.05f, 0.05f, 0.05f); 
     
    //Performing filtering and copy to cloud_filtered variable 
    vox_obj.filter(cloud_filtered);  
     //Create the filtering object
      pcl::PassThrough<pcl::PointXYZ> pass;
      pass.setInputCloud (cloud_filtered.makeShared());
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (0.0, 5.0);
      //pass.setFilterLimitsNegative (true);
      pass.filter (cloud_pass_throughed);
      pcl::toROSMsg(cloud_pass_throughed, output); 
      output.header.frame_id = "point_cloud"; 
       pcl_pub.publish(output);
    } 
 
protected: 
    ros::NodeHandle nh; 
    ros::Subscriber pcl_sub; 
    ros::Publisher pcl_pub; 
};
main(int argc, char** argv) 
{ 
    ros::init(argc, argv, "pcl_filter"); 
    ROS_INFO("Started Filter Node"); 
    cloudHandler handler; 
    ros::spin(); 
    return 0; 
} 