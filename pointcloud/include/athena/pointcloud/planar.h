#ifndef ATHENA_PLANAR_H
#define ATHENA_PLANAR_H

#include <ros/ros.h>
#include <Eigen/Core>
#include <athena/pointcloud/utils.h>

namespace athena{
  namespace pointcloud{

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

        ProcessPointCloudResult processPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double thresh);
    };
  };
};

#endif
