#ifndef TRANSFORM_CONVERSIONS_H
#define TRANSFORM_CONVERSIONS_H

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen_conversions/eigen_msg.h>

#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Core>


#define HOMOGENOUS_TRANFORM_ELEMENTS 16

namespace athena{
  namespace transform{

    // Various conversions
    tf::Transform pose_stamped_msg_to_tf(geometry_msgs::PoseStamped pose);


    // Various ways to transform poses or points
    geometry_msgs::PointStamped transform_point(tf2_ros::Buffer &tf_buffer, geometry_msgs::PointStamped point, std::string target_frame);
    geometry_msgs::PoseStamped transform_point(tf2_ros::Buffer &tf_buffer, geometry_msgs::PoseStamped pose, std::string source_frame, std::string target_frame);

    void publish_matrix_as_tf(tf::TransformBroadcaster &br, Eigen::Matrix4d transformation_matrix, std::string source, std::string dest);
    void publish_matrix_as_tf(tf::TransformBroadcaster &br, Eigen::Affine3d transformation_matrix, std::string source, std::string dest);

    Eigen::Vector3d translation_from_matrix(Eigen::Matrix4d matrix);
    Eigen::Vector3d euler_from_matrix(Eigen::Matrix4d matrix);
    Eigen::VectorXd xyzrpy_from_matrix(Eigen::Matrix4d matrix);

    Eigen::Quaternionf euler_to_quaternion(Eigen::Vector3d euler);
    Eigen::Vector3d quaternion_to_euler(Eigen::Quaterniond q);
    Eigen::Vector3d euler_from_rotation(Eigen::Matrix3d rot);

    Eigen::Matrix4d xyzrpy_to_matrix(Eigen::Vector3d xyz, Eigen::Vector3d rpy);
    Eigen::Matrix4d translation_matrix(double x, double y, double z);
    Eigen::Matrix4d euler_matrix(double roll, double pitch, double yaw);

    geometry_msgs::Point eigen3d_vector_to_point(Eigen::Vector3d vec);


    Eigen::Matrix4d array_to_eigen4d_matrix(const double transform[]);
    Eigen::Matrix4d array_to_eigen4d_matrix(const float transform[]);

    boost::array<double, HOMOGENOUS_TRANFORM_ELEMENTS> eigen4d_matrix_to_array(Eigen::Matrix4d transform);

  };
};

#endif
