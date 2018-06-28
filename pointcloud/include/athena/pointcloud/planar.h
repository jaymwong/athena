#ifndef ATHENA_PLANAR_H
#define ATHENA_PLANAR_H

#include <ros/ros.h>
#include <Eigen/Core>

namespace athena{
  namespace pointcloud{

    struct ClosestPointResult{
      Eigen::Vector3d closest_pt;
      double distance;
    };

    class PlanarModel{
      public:
        double a, b, c, d;
        PlanarModel(double a, double b, double c, double d);
        ~PlanarModel();

        ClosestPointResult getClosestPointOnPlane(Eigen::Vector3d point);
    };
  };
};

#endif
