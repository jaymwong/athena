#include "athena/transform/conversions.h"



tf::Transform athena::transform::pose_stamped_msg_to_tf(geometry_msgs::PoseStamped pose_msg){
  tf::Transform transform_result;
  transform_result.setOrigin(tf::Vector3(pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z));
  transform_result.setRotation(tf::Quaternion( pose_msg.pose.position.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w));
  return transform_result;
}


geometry_msgs::PointStamped athena::transform::transform_point(tf2_ros::Buffer &tf_buffer, geometry_msgs::PointStamped point, std::string target_frame){
  std::string source_frame = point.header.frame_id;
  geometry_msgs::PointStamped point_in_target_frame;

  tf_buffer.lookupTransform(source_frame, target_frame, ros::Time(0), ros::Duration(3.0));
  tf_buffer.transform(point, point_in_target_frame, target_frame);
  return point_in_target_frame;
}

// @param: pose - only using the position component to transform the point
// @param: source_frame
// @param: target_frame
geometry_msgs::PoseStamped athena::transform::transform_point(tf2_ros::Buffer &tf_buffer, geometry_msgs::PoseStamped pose, std::string source_frame, std::string target_frame){
  geometry_msgs::PointStamped point_in_source_frame;
  point_in_source_frame.point = pose.pose.position;
  point_in_source_frame.header.frame_id = source_frame;

  geometry_msgs::PointStamped point_in_target_frame = transform_point(tf_buffer, point_in_source_frame, target_frame);

  geometry_msgs::PoseStamped pose_in_target_frame;
  pose_in_target_frame.header.frame_id = target_frame;
  pose_in_target_frame.pose.position = point_in_target_frame.point;
  return pose_in_target_frame;
}

Eigen::Matrix4d athena::transform::translation_matrix(double x, double y, double z){
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  matrix.col(3) << x, y, z, 1.0;
  return matrix;
}

Eigen::Vector3d athena::transform::translation_from_matrix(Eigen::Matrix4d matrix){
  Eigen::Affine3d affine_matrix;
  affine_matrix.matrix() = matrix;
  return affine_matrix.translation();
}

Eigen::Vector3d athena::transform::euler_from_matrix(Eigen::Matrix4d matrix){
  auto euler = matrix.block<3,3>(0, 0).eulerAngles(2, 1, 0);
  Eigen::Vector3d result(euler(2), euler(1), euler(0));
  return result;
}

Eigen::Matrix4d athena::transform::euler_matrix(double roll, double pitch, double yaw){
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  tf::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  transform.setRotation(q);

  Eigen::Isometry3d tf_matrix;
  tf::transformTFToEigen(transform,  tf_matrix);
  return tf_matrix.matrix();
}

Eigen::Vector3d athena::transform::transform_point(Eigen::Matrix4d transform, Eigen::Vector3d pt){
  Eigen::Vector4d point(pt.x(), pt.y(), pt.z(), 1.0);
  return (transform * point).head<3>();
}

Eigen::Matrix4d athena::transform::xyzrpy_to_matrix(Eigen::Vector3d xyz, Eigen::Vector3d rpy){
  Eigen::Affine3d affine;
  affine.matrix() = athena::transform::euler_matrix(rpy(0), rpy(1), rpy(2));
  affine.translation() = xyz;
  return affine.matrix();
}

Eigen::Vector3d athena::transform::compute_midpoint(Eigen::Vector3d vec1, Eigen::Vector3d vec2){
  double x = vec1.x() + (vec2.x()-vec1.x())/2.0;
  double y = vec1.y() + (vec2.y()-vec1.y())/2.0;
  double z = vec1.z() + (vec2.z()-vec1.z())/2.0;
  Eigen::Vector3d result(x, y, z);
  return result;
}

Eigen::Matrix4d athena::transform::set_translation(Eigen::Matrix4d mat, Eigen::Vector3d vec){
  mat.col(3).head<3>() = vec;
  return mat;
}

Eigen::Matrix4d athena::transform::xyzrpy_to_matrix(std::vector<double> xyzrpy) {
  Eigen::Affine3d affine;
  affine.matrix() = athena::transform::euler_matrix(xyzrpy[3], xyzrpy[4], xyzrpy[5]);
  affine.translation().x() = xyzrpy[0];
  affine.translation().y() = xyzrpy[1];
  affine.translation().z() = xyzrpy[2];
  return affine.matrix();
}

Eigen::VectorXd athena::transform::xyzrpy_from_matrix(Eigen::Matrix4d matrix){
  Eigen::VectorXd xyzrpy(6);
  auto xyz = athena::transform::translation_from_matrix(matrix);
  auto rpy = athena::transform::euler_from_matrix(matrix);
  xyzrpy << xyz, rpy;
  return xyzrpy;
}


Eigen::Vector3d athena::transform::diff_vector(Eigen::Vector3d v1, std::vector<double> v2){
  Eigen::Vector3d result(v1.x()-v2[0], v1.y()-v2[1], v1.z()-v2[2]);
  return result;
}

boost::array<double, 3> athena::transform::to_boost_arrayd(Eigen::Vector3d vec){
  boost::array<double, 3> result;
  result[0] = vec.x();
  result[1] = vec.y();
  result[2] = vec.z();
  return result;
}

std::vector<float> athena::transform::to_std_vectorf(Eigen::Vector3d mat){
  std::vector<float> vec(mat.data(), mat.data() + mat.rows() * mat.cols());
  return vec;
}

Eigen::Vector3d athena::transform::quaternion_to_euler(Eigen::Quaterniond q){
  Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
  return euler;
}

Eigen::Quaternionf athena::transform::euler_to_quaternion(Eigen::Vector3d euler){
  Eigen::Quaternionf q;
  q = Eigen::AngleAxisf(euler(0), Eigen::Vector3f::UnitX())
    * Eigen::AngleAxisf(euler(1), Eigen::Vector3f::UnitY())
    * Eigen::AngleAxisf(euler(2), Eigen::Vector3f::UnitZ());
  return q;
}

Eigen::Vector3d athena::transform::euler_from_rotation(Eigen::Matrix3d rot){
  return rot.eulerAngles(0, 1, 2);
}

geometry_msgs::Point athena::transform::eigen3d_vector_to_point(Eigen::Vector3d vec){
  geometry_msgs::Point result;
  result.x = vec.x();
  result.y = vec.y();
  result.z = vec.z();
  return result;
}

void athena::transform::publish_matrix_as_tf(tf::TransformBroadcaster &br, Eigen::Matrix4d transformation_matrix, std::string source, std::string dest){
  Eigen::Affine3d affine;
  affine.matrix() = transformation_matrix;
  tf::Transform t;
  tf::transformEigenToTF(affine, t);
  auto q = t.getRotation().normalize();
  t.setRotation(q);
  br.sendTransform(tf::StampedTransform(t, ros::Time::now(), source, dest));
}

void athena::transform::publish_matrix_as_tf(tf::TransformBroadcaster &br, Eigen::Affine3d transformation_matrix, std::string source, std::string dest){
  publish_matrix_as_tf(br, transformation_matrix.matrix(), source, dest);
}

Eigen::Matrix4d athena::transform::array_to_eigen4d_matrix(const float transform[]){
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

geometry_msgs::Pose athena::transform::to_geometry_msg_pose(Eigen::Vector3d vec){
  geometry_msgs::Pose pose;
  pose.position.x = vec.x();
  pose.position.y = vec.y();
  pose.position.z = vec.z();
  pose.orientation.w = 1.0;
  return pose;
}

Eigen::Vector3d athena::transform::to_eigen_vector3d(geometry_msgs::Pose pose){
  Eigen::Vector3d vec(pose.position.x, pose.position.y, pose.position.z);
  return vec;
}

Eigen::Matrix4d athena::transform::array_to_eigen4d_matrix(const double transform[]){
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

boost::array<double, HOMOGENOUS_TRANFORM_ELEMENTS> athena::transform::eigen4d_matrix_to_array(Eigen::Matrix4d transform){
  boost::array<double, HOMOGENOUS_TRANFORM_ELEMENTS> transform_array;
  Eigen::MatrixXd resize_transform = transform;
  resize_transform.resize(1, HOMOGENOUS_TRANFORM_ELEMENTS);

  for (int i = 0; i < HOMOGENOUS_TRANFORM_ELEMENTS; i++){
    transform_array[i] = resize_transform(0, i);
  }
  return transform_array;
}

Matrix4dStatistics athena::transform::compute_transform_statistics(std::vector<Eigen::Matrix4d> transforms) {
  std::vector<double> x, y, z, r, p, yw;
  Matrix4dStatistics transform_statistics;
  for (int i = 0; i < transforms.size(); i++) {
    auto xyz = athena::transform::translation_from_matrix(transforms[i]);
    auto rpy = athena::transform::euler_from_matrix(transforms[i]);
    x.push_back(xyz[0]);
    y.push_back(xyz[1]);
    z.push_back(xyz[2]);
    r.push_back(rpy[0]);
    p.push_back(rpy[1]);
    yw.push_back(rpy[2]);
  }
  std::sort (x.begin(), x.end());
  std::sort (y.begin(), y.end());
  std::sort (z.begin(), z.end());
  std::sort (r.begin(), r.end());
  std::sort (p.begin(), p.end());
  std::sort (yw.begin(), yw.end());
  transform_statistics.mean.push_back(findmean(x));
  transform_statistics.mean.push_back(findmean(y));
  transform_statistics.mean.push_back(findmean(z));
  transform_statistics.mean.push_back(findmean(r));
  transform_statistics.mean.push_back(findmean(p));
  transform_statistics.mean.push_back(findmean(yw));
  transform_statistics.median.push_back(findmedian(x));
  transform_statistics.median.push_back(findmedian(y));
  transform_statistics.median.push_back(findmedian(z));
  transform_statistics.median.push_back(findmedian(r));
  transform_statistics.median.push_back(findmedian(p));
  transform_statistics.median.push_back(findmedian(yw));
  transform_statistics.std.push_back(findstd(x, transform_statistics.mean[0]));
  transform_statistics.std.push_back(findstd(y, transform_statistics.mean[1]));
  transform_statistics.std.push_back(findstd(z, transform_statistics.mean[2]));
  transform_statistics.std.push_back(findstd(r, transform_statistics.mean[3]));
  transform_statistics.std.push_back(findstd(p, transform_statistics.mean[4]));
  transform_statistics.std.push_back(findstd(yw, transform_statistics.mean[5]));
  return transform_statistics;
}

double athena::transform::findmean(std::vector<double> val) {
  double sum = 0;
  for (int i = 0; i < val.size(); i++) {
    sum += val[i];
  }
  return double(sum/val.size());
}

double athena::transform::findmedian(std::vector<double> val) {
  if(val.size()%2 == 0)
    return double((val[val.size()/2] + val[(val.size()/2) - 1]) / 2);
  else if(val.size()%2 == 0)
    return double(val[(val.size()/2)]);
}

double athena::transform::findstd(std::vector<double> val, double mean) {
  std::vector<double> diff;
  std::cout << val[val.size() - 1] - val[0] << std::endl;
  for (int i = 0; i < val.size(); i++) {
    diff.push_back(val[i] - mean);
  }
  return findmean(diff);
}
