#ifndef TRANSFORM_CONVERSIONS_H
#define TRANSFORM_CONVERSIONS_H

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen_conversions/eigen_msg.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Core>

struct Matrix4dStatistics {
  std::vector<double> mean, median, std;
};

namespace athena{

  namespace conversions{
    Eigen::Matrix4d toEigenMatrix4d(const double transform[]);
    Eigen::Matrix4d toEigenMatrix4d(const float transform[]);

    geometry_msgs::Pose toGeometryMsgPose(Eigen::Vector3d vec);
    geometry_msgs::Pose toGeometryMsgPose(Eigen::Vector3d vec, Eigen::Quaternionf quat);
    geometry_msgs::Point toGeometryMsgPoint(Eigen::Vector3d vec);
    geometry_msgs::Point toGeometryMsgPoint(boost::array<float, 3> array);

    Eigen::Vector3d toEigenVector3d(geometry_msgs::Pose pose);
    Eigen::Vector3d toEigenVector3d(geometry_msgs::PoseStamped pt);
    Eigen::VectorXd toEigenVectorXd(std::vector<double> input);

    boost::array<double, 16> toBoostArrayd(Eigen::Matrix4d transform);
    boost::array<double, 3> toBoostArray3d(Eigen::Vector3d vec);
    std::vector<float> toStdVectorf(Eigen::Vector3d mat);

    tf::Transform toTfTransform(geometry_msgs::PoseStamped pose);

    std::string toString(Eigen::Vector3d vec);
    std::string toString(std::vector<double> v);
    std::string toString(std::vector<int> v);
  };

  namespace transform{

    // Various ways to transform poses or points
    geometry_msgs::PointStamped transform_point(tf2_ros::Buffer &tf_buffer, geometry_msgs::PointStamped point, std::string target_frame);
    geometry_msgs::PoseStamped transform_point(tf2_ros::Buffer &tf_buffer, geometry_msgs::PoseStamped pose, std::string source_frame, std::string target_frame);

    void publish_matrix_as_tf(tf::TransformBroadcaster &br, Eigen::Matrix4d transformation_matrix, std::string root, std::string name);
    void publish_matrix_as_tf(tf::TransformBroadcaster &br, Eigen::Affine3d transformation_matrix, std::string root, std::string name);

    Eigen::Vector3d translation_from_matrix(Eigen::Matrix4d matrix);
    Eigen::Vector3d euler_from_matrix(Eigen::Matrix4d matrix);
    Eigen::VectorXd xyzrpy_from_matrix(Eigen::Matrix4d matrix);

    Eigen::Quaternionf euler_to_quaternion(Eigen::Vector3d euler);
    Eigen::Quaternionf euler_to_quaternion(double roll, double pitch, double yaw);
    Eigen::Vector3d quaternion_to_euler(Eigen::Quaterniond q);
    Eigen::Vector3d euler_from_rotation(Eigen::Matrix3d rot);

    Eigen::Matrix4d xyzrpy_to_matrix(Eigen::Vector3d xyz, Eigen::Vector3d rpy);
    Eigen::Matrix4d xyzrpy_to_matrix(std::vector<double> xyzrpy);
    Eigen::Matrix4d translation_matrix(double x, double y, double z);
    Eigen::Matrix4d translation_matrix(Eigen::Vector3d xyz);

    Eigen::Matrix4d euler_matrix(double roll, double pitch, double yaw);
    Eigen::Matrix4d euler_matrix(Eigen::Vector3d rpy);
    Eigen::Matrix4d quaternion_matrix(double x, double y, double z, double w);

    Eigen::Vector3d compute_midpoint(Eigen::Vector3d vec1, Eigen::Vector3d vec2);
    Eigen::Vector3d transform_point(Eigen::Matrix4d transform, Eigen::Vector3d pt);
    Eigen::Matrix4d set_translation(Eigen::Matrix4d mat, Eigen::Vector3d vec);

    Eigen::Vector3d diff_vector(Eigen::Vector3d v1, std::vector<double> v2);

    Matrix4dStatistics compute_transform_statistics(std::vector<Eigen::Matrix4d> transforms);
    double findmean(std::vector<double> val);
    double findmedian(std::vector<double> val);
    double findstd(std::vector<double> val, double mean);
  };
};

#endif
