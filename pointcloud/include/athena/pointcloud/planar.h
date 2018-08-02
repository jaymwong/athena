#ifndef ATHENA_PLANAR_H
#define ATHENA_PLANAR_H

#include <ros/ros.h>
#include <Eigen/Core>
#include <athena/pointcloud/utils.h>
#include <athena/transform/conversions.h>
#include <athena/pointcloud/conversions.h>
#include <athena_msgs/PlanarModel.h>

namespace athena{
  namespace pointcloud{

    // Processing the planar model with plane removal and distance functions
    struct ClosestPointResult{
      Eigen::Vector3d pt;
      pcl::PointXYZ pcl_pt;
      double distance, signed_distance;
    };

    struct ProcessPointCloudResult{
      pcl::PointCloud<pcl::PointXYZ>::Ptr inliers, outliers;
    };

    class PlanarModel{
      public:
        double a, b, c, d;
        PlanarModel(double a, double b, double c, double d);
        ~PlanarModel();

        ClosestPointResult getClosestPointOnPlane(Eigen::Vector3d point);
        ClosestPointResult getClosestPointOnPlane(pcl::PointXYZ point);

        int getPointRelativeDirection(pcl::PointXYZ point);
        int getPointRelativeDirection(Eigen::Vector3d point);

        ProcessPointCloudResult processPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double thresh, Eigen::Matrix4d to_world_frame=Eigen::Matrix4d::Identity());
    };


    struct PlaneEstimationResult{
      bool success;
      pcl::ModelCoefficients::Ptr coefficients;
      pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud;
    };

    PlaneEstimationResult estimatePlanarModel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,  double thresh);
    athena::pointcloud::PlanarModel* createPlanarModelFromCoefficients(pcl::ModelCoefficients::Ptr coefficients);

  };

  namespace conversions{
    athena_msgs::PlanarModel toPlanarModelMsg(athena::pointcloud::PlanarModel *model, std::string frame_id);
    athena::pointcloud::PlanarModel* fromPlanarModelmsg(athena_msgs::PlanarModel msg);

  };
};

#endif
