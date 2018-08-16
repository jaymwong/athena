#ifndef ATHENA_POINTCLOUD_CROPPING_H
#define ATHENA_POINTCLOUD_CROPPING_H

#include <athena/pointcloud/utils.h>

namespace athena{
  namespace pointcloud{

    // Passthrough filters in each dimension to crop the cloud; generally done in the world frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr doPassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string axis, double min, double max);
    pcl::PointCloud<pcl::PointXYZ>::Ptr doPassThroughCubeCrop(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ pt, double radius);
    pcl::PointCloud<pcl::PointXYZ>::Ptr doPassThroughCubeCrop(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Matrix4d wTc, pcl::PointXYZ pt, double radius);

    // Transforms the cloud into the world frame and removes all points under world z=0
    pcl::PointCloud<pcl::PointXYZ>::Ptr removeNegativeWorldPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, Eigen::Matrix4d cam_to_world);

    // Removes a sub-cloud from the original cloud; assumes that all points in subclout is a subset of original
    pcl::PointCloud<pcl::PointXYZ>::Ptr removeSubCloudFromOriginalCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud_, pcl::PointXYZ pt, double radius_x, double radius_y, double radius_z);

    // Removes all points outside of some std-dev of the cloud; will return a sub-set of the points in a new cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr getCloudPointsWithinStdDev(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, double thresh);

  };
};

#endif
