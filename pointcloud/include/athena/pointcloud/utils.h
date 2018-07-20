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

// for finding AABB and OBB
#include <pcl/features/moment_of_inertia_estimation.h>

#include <athena/transform/conversions.h>
// Athena supported message types
#include <athena_pointcloud/CloudGeometries.h>

#include "athena/pointcloud/conversions.h"

#define kInfinity 9999999

struct PointCloudProperties{
  pcl::PointXYZ min_point, max_point;
};

struct BoundingBoxGeometry {
  Eigen::Vector3d AABB_dimensions;
  Eigen::Vector3d OBB_dimensions;
  Eigen::Affine3d transformation_world_to_OBB;
  double yaw;
  visualization_msgs::Marker bounding_box;
};

namespace athena {
  namespace pointcloud{

    // A class with data structures for the object geometries
    class ObjectGeometries{
      public:
        std::string obj_name;
        double min_x, max_x, min_y, max_y, min_z, max_z;

        ObjectGeometries(std::string n, double mix, double max, double miy, double may, double miz, double maz){
          obj_name = n;
          min_x = mix;
          max_x = max;
          min_y = miy;
          max_y = may;
          min_z = miz;
          max_z = maz;
        };
        ~ObjectGeometries(){};

        void printGeometries(){
          std::cout << obj_name << ":\n     x:" << min_x << " " << max_x << "\n     y:" << min_y << " " << max_y << "\n     z:" << min_z << " " << max_z << "\n";
        }

        athena_pointcloud::CloudGeometry toCloudGeometryMsg(){
          athena_pointcloud::CloudGeometry geometry;
          geometry.id = obj_name;
          geometry.x[0] = min_x;
          geometry.x[1] = max_x;
          geometry.y[0] = min_y;
          geometry.y[1] = max_y;
          geometry.z[0] = min_z;
          geometry.z[1] = max_z;
          return geometry;
        }
    };


    Eigen::Vector3d computePointCloudMedian(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    Eigen::Vector3d computePointCloudMedian(std::vector<double> cluster_pt_x, std::vector<double> cluster_pt_y, std::vector<double> cluster_pt_z);
    Eigen::Vector3d computePointCloudMedian(std::vector<Eigen::Vector3d> vec);

    Eigen::Vector3d computePointCloudBoundingBoxOrigin(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    std::vector<double> getMinAndMaxFromVector(std::vector<double> my_vector);

    // Computes the geometry (e.g. min and max displacements aloong the world coordinate frame) for a point cloud
    athena::pointcloud::ObjectGeometries* computePointCloudGeometries(std::string obj_name, pcl::PointCloud<pcl::PointXYZ> cloud);

    sensor_msgs::PointCloud2 toSensorMsgPointCloud2(pcl::PointCloud<pcl::PointXYZ> pcl_cloud);
    sensor_msgs::PointCloud2 toSensorMsgPointCloud2(pcl::PointCloud<pcl::PointXYZRGBA> pcl_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr sensorMsgToPclPointCloudXYZ(const sensor_msgs::PointCloud2ConstPtr& input);


    void publishPointCloudXYZ(ros::Publisher pub, pcl::PointCloud<pcl::PointXYZ> &pcl_cloud, std::string frame_id);
    void publishPointCloudXYZRGB(ros::Publisher pub, pcl::PointCloud<pcl::PointXYZRGB> &pcl_cloud, std::string frame_id);

    PointCloudProperties computePointCloudMinMax(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    BoundingBoxGeometry obtainBoundingBoxGeomtry (pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
    double computeBoundingBoxYaw(Eigen::Matrix3f rotation_matrix, Eigen::Vector3d position, Eigen::Vector3d AABB_dimensions, Eigen::Vector3d OBB_dimensions);
    visualization_msgs::Marker createVisualizationMarker(Eigen::Vector3d OBB_dimensions, Eigen::Vector3d center, double yaw);
    std::vector<int> sortAABBDimensions(Eigen::Vector3d AABB_dimensions);
    std::vector<Eigen::Vector3d> transformToWorldCoordinates(tf2_ros::Buffer &tf_buffer, Eigen::Vector3d OBB_dimensions, std::string source_frame);

    pcl::PointXYZ eigenVectorToPclPointXYZ(Eigen::Vector3d vector);
    geometry_msgs::Pose pclPointXYZToGeometryMsgPose(pcl::PointXYZ pt);

    pcl::PointCloud<pcl::PointXYZ>::Ptr doPassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string axis, double min, double max);
    pcl::PointCloud<pcl::PointXYZ>::Ptr doPassThroughCubeCrop(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ pt, double radius);

    pcl::PointIndices::Ptr findCloudInliers(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ pt, double radius_x, double radius_y, double radius_z);

    pcl::PointCloud<pcl::PointXYZ>::Ptr removeNegativeWorldPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, Eigen::Matrix4d cam_to_world);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr createColorizedPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int r, int g, int b);

    pcl::PointCloud<pcl::PointXYZ>::Ptr convertClusterToPointCloud(int idx, std::vector <pcl::PointIndices> clusters, pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr getMaxEuclideanClusterFromPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, double tolerance);
    pcl::PointCloud<pcl::PointXYZ>::Ptr getCloudPointsWithinStdDev(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, double thresh);


    pcl::PointCloud<pcl::PointXYZ>::Ptr doNeighborRadiusSearch(pcl::PointXYZ searchPoint, pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree_flann,
                                                               pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud, double radius);


  };
};

#endif
