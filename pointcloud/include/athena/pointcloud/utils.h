#ifndef POINTCLOUD_BOUNDING_GEOMETRY_H
#define POINTCLOUD_BOUNDING_GEOMETRY_H

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
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>

// Various PCL io headers
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/ascii_io.h>
#include <pcl/io/vtk_lib_io.h>

// Various PCL filters
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/icp.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/geometry/triangle_mesh.h>

#include <athena/transform/conversions.h>
#include <athena/pointcloud/conversions.h>

#define kInfinity 9999999

struct PointCloudProperties{
  pcl::PointXYZ min_point, max_point;
  Eigen::Vector2d x, y, z;
};

namespace athena {
  namespace pointcloud{

    pcl::PointCloud<pcl::PointXYZ>::Ptr curatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    Eigen::Vector3d computePointCloudMedian(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    Eigen::Vector3d computePointCloudMedian(std::vector<double> cluster_pt_x, std::vector<double> cluster_pt_y, std::vector<double> cluster_pt_z);
    Eigen::Vector3d computePointCloudMedian(std::vector<Eigen::Vector3d> vec);

    std::vector<double> getMinAndMaxFromVector(std::vector<double> my_vector);

    // Naively projects the pointcloud into the z=0 plane; squashes the cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr naivePointCloudProjection(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    // Publishing helper function
    void publishPointCloudXYZ(ros::Publisher pub, pcl::PointCloud<pcl::PointXYZ> &pcl_cloud, std::string frame_id);
    void publishPointCloudXYZRGB(ros::Publisher pub, pcl::PointCloud<pcl::PointXYZRGB> &pcl_cloud, std::string frame_id);

    // Directly just computing the min and max from the input cloud and calling that a tentative bounding geometry
    PointCloudProperties computePointCloudMinMax(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    pcl::PointIndices::Ptr findCloudInliers(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ pt, double radius_x, double radius_y, double radius_z);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr createColorizedPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int r, int g, int b);

    pcl::PointCloud<pcl::PointXYZ>::Ptr convertClusterToPointCloud(int idx, std::vector <pcl::PointIndices> clusters, pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr getMaxEuclideanClusterFromPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, double tolerance);

    pcl::PointCloud<pcl::PointXYZ>::Ptr doNeighborRadiusSearch(pcl::PointXYZ searchPoint, pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree_flann,
                                                               pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud, double radius);


  };
};

#endif
