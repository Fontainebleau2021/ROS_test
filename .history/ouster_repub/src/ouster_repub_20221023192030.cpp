#include <ros/ros.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>

ros::Publisher pub;

struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

struct OusterPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
    (uint8_t, ring, ring) (uint16_t, noise, noise) (uint32_t, range, range)
)

// Use the Velodyne point format as a common representation
using PointXYZIRT = VelodynePointXYZIRT;

pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn;
sensor_msgs::PointCloud2 currentCloudMsg;

void 
cloud_show (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  sensor_msgs::PointCloud2 output;
  currentCloudMsg = std::move(*cloud_msg);
  pcl::moveFromROSMsg(currentCloudMsg, *tmpOusterCloudIn);
  laserCloudIn->points.resize(tmpOusterCloudIn->size());
  laserCloudIn->is_dense = tmpOusterCloudIn->is_dense;
    for (size_t i = 0; i < tmpOusterCloudIn->size(); i++)
    {
        auto &src = tmpOusterCloudIn->points[i];
        auto &dst = laserCloudIn->points[i];
        dst.x = src.x;
        dst.y = src.y;
        dst.z = src.z;
        dst.intensity = src.intensity;
        dst.ring = src.ring;
        dst.time = src.t * 1e-9f;
    }

   pcl::toROSMsg(*laserCloudIn, output);
  //output = currentCloudMsg;
  // Publish the data
  pub.publish (output);
}

int main (int argc, char** argv)
{
laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
 tmpOusterCloudIn.reset(new pcl::PointCloud<OusterPointXYZIRT>());
  // Initialize ROS
  ros::init (argc, argv, "ouster_repub");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("points_raw", 1, cloud_show);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("velodyne_points", 1);

  // Spin
  ros::spin ();
}