#ifndef POINTCLOUD_CONVERSIONS_UTILS_H
#define POINTCLOUD_CONVERSIONS_UTILS_H

#include <ros/ros.h>
#include <ros/package.h>

#include <Eigen/Core>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>


struct PointCloudProperties{
  pcl::PointXYZ min_point, max_point;
};

namespace pointcloud_utils{

  Eigen::Vector3d computePointCloudMedian(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  Eigen::Vector3d computePointCloudMedian(std::vector<double> cluster_pt_x, std::vector<double> cluster_pt_y, std::vector<double> cluster_pt_z);

  void publishPointCloudXYZ(ros::Publisher pub, pcl::PointCloud<pcl::PointXYZ> &pcl_cloud, std::string frame_id);
  void publishPointCloudXYZRGB(ros::Publisher pub, pcl::PointCloud<pcl::PointXYZRGB> &pcl_cloud, std::string frame_id);

  PointCloudProperties computePointCloudMinMax(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr createColorizedPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int r, int g, int b);

};

#endif
