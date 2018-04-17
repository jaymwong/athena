#ifndef POINTCLOUD_CONVERSIONS_CONVERSIONS_H
#define POINTCLOUD_CONVERSIONS_CONVERSIONS_H

#include <ros/ros.h>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>


namespace pointcloud_conversions{

  void pointcloudToBGRImage(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, cv::Mat &coords, cv::Mat &image);
  Eigen::Vector3d pclPointToEigenVector3d(pcl::PointXYZ pt);

};

#endif
