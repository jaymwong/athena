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
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>


namespace athena{
  namespace pointcloud{

    void pointcloudToBGRImage(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, cv::Mat &coords, cv::Mat &image);
  };

  namespace conversions{

    Eigen::Vector3d toEigenVector3d(pcl::PointXYZ pt);
    geometry_msgs::Pose toGeometryMsgPose(pcl::PointXYZ pt);

    pcl::PointXYZ toPclPointXYZ(Eigen::Vector3d vec);
    pcl::PointXYZ toPclPointXYZ(geometry_msgs::PointStamped msg);

    sensor_msgs::PointCloud2 toSensorMsgPointCloud2(pcl::PointCloud<pcl::PointXYZ> pcl_cloud);
    sensor_msgs::PointCloud2 toSensorMsgPointCloud2(pcl::PointCloud<pcl::PointXYZRGBA> pcl_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr toPclPointCloudXYZ(const sensor_msgs::PointCloud2ConstPtr& input);
    pcl::PointCloud<pcl::PointXYZ>::Ptr toPclPointCloudXYZ(sensor_msgs::PointCloud2 input);
  };
};

#endif
