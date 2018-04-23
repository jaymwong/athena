#include "athena/transform/conversions.h"



tf::Transform transform_conversions::pose_stamped_msg_to_tf(geometry_msgs::PoseStamped pose_msg){
  tf::Transform transform_result;
  transform_result.setOrigin(tf::Vector3(pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z));
  transform_result.setRotation(tf::Quaternion( pose_msg.pose.position.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w));
  return transform_result;
}


geometry_msgs::PointStamped transform_conversions::transform_point(tf2_ros::Buffer &tf_buffer, geometry_msgs::PointStamped point, std::string target_frame){
  std::string source_frame = point.header.frame_id;
  geometry_msgs::PointStamped point_in_target_frame;

  tf_buffer.lookupTransform(source_frame, target_frame, ros::Time(0), ros::Duration(3.0));
  tf_buffer.transform(point, point_in_target_frame, target_frame);
  return point_in_target_frame;
}

// @param: pose - only using the position component to transform the point
// @param: source_frame
// @param: target_frame
geometry_msgs::PoseStamped transform_conversions::transform_point(tf2_ros::Buffer &tf_buffer, geometry_msgs::PoseStamped pose, std::string source_frame, std::string target_frame){
  geometry_msgs::PointStamped point_in_source_frame;
  point_in_source_frame.point = pose.pose.position;
  point_in_source_frame.header.frame_id = source_frame;

  geometry_msgs::PointStamped point_in_target_frame = transform_point(tf_buffer, point_in_source_frame, target_frame);

  geometry_msgs::PoseStamped pose_in_target_frame;
  pose_in_target_frame.header.frame_id = target_frame;
  pose_in_target_frame.pose.position = point_in_target_frame.point;
  return pose_in_target_frame;
}

Eigen::Matrix4d transform_conversions::translation_matrix(double x, double y, double z){
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  matrix.col(3) << x, y, z, 1.0;
  return matrix;
}


Eigen::Vector3d transform_conversions::translation_from_matrix(Eigen::Matrix4d matrix){
  Eigen::Affine3d affine_matrix;
  affine_matrix.matrix() = matrix;
  return affine_matrix.translation();
}

Eigen::Vector3d transform_conversions::euler_from_matrix(Eigen::Matrix4d matrix){
  auto euler = matrix.block<3,3>(0, 0).eulerAngles(2, 1, 0);
  Eigen::Vector3d result(euler(2), euler(1), euler(0));
  return result;
}

Eigen::Matrix4d transform_conversions::euler_matrix(double roll, double pitch, double yaw){
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  tf::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  transform.setRotation(q);

  Eigen::Isometry3d tf_matrix;
  tf::transformTFToEigen(transform,  tf_matrix);
  return tf_matrix.matrix();
}

Eigen::Vector3d transform_conversions::quaternion_to_euler(Eigen::Quaterniond q){
  Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
  return euler;
}

Eigen::Quaternionf transform_conversions::euler_to_quaternion(Eigen::Vector3d euler){
  Eigen::Quaternionf q;
  q = Eigen::AngleAxisf(euler(0), Eigen::Vector3f::UnitX())
    * Eigen::AngleAxisf(euler(1), Eigen::Vector3f::UnitY())
    * Eigen::AngleAxisf(euler(2), Eigen::Vector3f::UnitZ());
  return q;
}

Eigen::Vector3d transform_conversions::euler_from_rotation(Eigen::Matrix3d rot){
  return rot.eulerAngles(0, 1, 2);
}

geometry_msgs::Point transform_conversions::eigen3d_vector_to_point(Eigen::Vector3d vec){
  geometry_msgs::Point result;
  result.x = vec.x();
  result.y = vec.y();
  result.z = vec.z();
  return result;
}

void transform_conversions::publish_matrix_as_tf(tf::TransformBroadcaster &br, Eigen::Matrix4d transformation_matrix, std::string source, std::string dest){
  Eigen::Affine3d *eigen_transform = new Eigen::Affine3d();
  eigen_transform->matrix() = transformation_matrix;
  tf::Transform t;
  tf::transformEigenToTF(*eigen_transform, t);
  auto q = t.getRotation().normalize();
  t.setRotation(q);
  br.sendTransform(tf::StampedTransform(t, ros::Time::now(), source, dest));
}

void transform_conversions::publish_matrix_as_tf(tf::TransformBroadcaster &br, Eigen::Affine3d transformation_matrix, std::string source, std::string dest){
  publish_matrix_as_tf(br, transformation_matrix.matrix(), source, dest);
}

Eigen::Matrix4d transform_conversions::array_to_eigen4d_matrix(const float transform[]){
  Eigen::MatrixXd obj_pose;
  obj_pose.resize(HOMOGENOUS_TRANFORM_ELEMENTS, 1);
  for (int i = 0; i < HOMOGENOUS_TRANFORM_ELEMENTS; i++){
    obj_pose(i, 0) = transform[i];
  }
  obj_pose.resize(4, 4);

  // Assert the bottom row is 0 0 0 1; otherwise transpose the 4x4 matrix
  if (obj_pose(3,0) != 0.0 || obj_pose(3,1) != 0.0 || obj_pose(3,2) != 0.0){
    //std::cout << "Transposing the 4x4 matrix!\n";
    Eigen::MatrixXd obj_pose_transpose = obj_pose.transpose();
    obj_pose = obj_pose_transpose;
  }
  return obj_pose;
}

Eigen::Matrix4d transform_conversions::array_to_eigen4d_matrix(const double transform[]){
  Eigen::MatrixXd obj_pose;
  obj_pose.resize(HOMOGENOUS_TRANFORM_ELEMENTS, 1);
  for (int i = 0; i < HOMOGENOUS_TRANFORM_ELEMENTS; i++){
    obj_pose(i, 0) = transform[i];
  }
  obj_pose.resize(4, 4);

  // Assert the bottom row is 0 0 0 1; otherwise transpose the 4x4 matrix
  if (obj_pose(3,0) != 0.0 || obj_pose(3,1) != 0.0 || obj_pose(3,2) != 0.0){
    //std::cout << "Transposing the 4x4 matrix!\n";
    Eigen::MatrixXd obj_pose_transpose = obj_pose.transpose();
    obj_pose = obj_pose_transpose;
  }
  return obj_pose;
}

boost::array<double, HOMOGENOUS_TRANFORM_ELEMENTS> transform_conversions::eigen4d_matrix_to_array(Eigen::Matrix4d transform){
  boost::array<double, HOMOGENOUS_TRANFORM_ELEMENTS> transform_array;
  Eigen::MatrixXd resize_transform = transform;
  resize_transform.resize(1, HOMOGENOUS_TRANFORM_ELEMENTS);

  for (int i = 0; i < HOMOGENOUS_TRANFORM_ELEMENTS; i++){
    transform_array[i] = resize_transform(0, i);
  }
  return transform_array;
}
