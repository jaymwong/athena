#ifndef POINTCLOUD_CONVERSIONS_UTILS_H
#define POINTCLOUD_CONVERSIONS_UTILS_H

#include <ros/ros.h>
#include <ros/package.h>

#include <gtest/gtest.h>
#include <fstream>
#include <locale>
#include <stdexcept>

#include <Eigen/Core>
#include <cmath>

// Core PCL headers as well as message types
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_traits.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/console/print.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_broadcaster.h>

// for finding AABB and OBB
#include <pcl/features/moment_of_inertia_estimation.h>

#include <athena/transform/conversions.h>
// Athena supported message types
#include <athena_pointcloud/CloudGeometries.h>

#include "athena/pointcloud/conversions.h"
#include "athena/pointcloud/utils.h"
#include "athena/pointcloud/planar.h"

struct BoundingBoxGeometry {
  Eigen::Vector3d AABB_dimensions;
  Eigen::Vector3d OBB_dimensions;
  Eigen::Affine3d transformation_world_to_OBB;
  double yaw;
  visualization_msgs::Marker bounding_box;
};

namespace athena {
  namespace pointcloud{
    Eigen::Vector3d computePointCloudBoundingBoxOrigin(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    BoundingBoxGeometry obtainBoundingBoxGeomtry (pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, ros::Publisher pub_transformed_cloud);
    double computeBoundingBoxYaw(Eigen::Matrix3f rotation_matrix, Eigen::Vector3d position, Eigen::Vector3d AABB_dimensions, Eigen::Vector3d OBB_dimensions);
    visualization_msgs::Marker createVisualizationMarker(Eigen::Vector3d OBB_dimensions, Eigen::Vector3d center, double yaw);
    std::vector<int> sortAABBDimensions(Eigen::Vector3d AABB_dimensions);
    Eigen::Matrix4f rotateFrameAlongWorldX(Eigen::Vector3d OBB_dimensions, Eigen::Vector3d AABB_dimensions, Eigen::Matrix4d transform, double angle);
    Eigen::Matrix4f rotateFrameAlongWorldY(Eigen::Vector3d OBB_dimensions, Eigen::Vector3d AABB_dimensions, Eigen::Matrix4d transform, double angle);
    std::vector<Eigen::Vector3d> transformToWorldCoordinates(Eigen::Vector3d OBB_dimensions, Eigen::Matrix4d transform);
  };
};

#endif
