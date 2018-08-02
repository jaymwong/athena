#include "athena/pointcloud/bounding_geometry.h"

BoundingBoxGeometry athena::pointcloud::obtainBoundingBoxGeometry (pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, ros::Publisher pub_transformed_cloud, BoundingRequest req_params) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  BoundingBoxGeometry box_geometry;
  pcl::PointXYZ min_point_OBB, max_point_OBB, min_point_AABB, max_point_AABB;
  pcl::PointXYZ min_point_world, max_point_world, position_OBB;
  Eigen::Matrix3f rotational_matrix_OBB;
  tf::TransformBroadcaster br_;

  pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
  feature_extractor.setInputCloud(input_cloud);
  feature_extractor.compute();

  feature_extractor.getAABB(min_point_AABB, max_point_AABB);
  feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

  auto obb_min_point = athena::conversions::toEigenVector3d(min_point_OBB);
  auto obb_max_point = athena::conversions::toEigenVector3d(max_point_OBB);
  auto aabb_min_point = athena::conversions::toEigenVector3d(min_point_AABB);
  auto aabb_max_point = athena::conversions::toEigenVector3d(max_point_AABB);
  auto position = athena::conversions::toEigenVector3d(position_OBB);

  box_geometry.AABB_dimensions = aabb_max_point - aabb_min_point;
  box_geometry.OBB_dimensions = obb_max_point - obb_min_point;

  // the transform gives the matrix to get the box frame from the world frame
  Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
  projectionTransform.block<3,3>(0,0) = rotational_matrix_OBB;
  projectionTransform.block<3,1>(0,3) = position.cast <float> ();
  box_geometry.transformation_world_to_OBB.matrix() = projectionTransform.cast <double> ();

  box_geometry.yaw  = computeBoundingBoxYaw(rotational_matrix_OBB, position, box_geometry.AABB_dimensions, box_geometry.OBB_dimensions);

  // to retrieve the actual box dimensions in world X, Y, Z- axis the cloud is rotated by -yaw and then min-max is performed using which the box dimensions
  // and center of the box in world frame are found
  pcl::transformPointCloud(*input_cloud, *transformed_cloud, athena::transform::translation_matrix(-position[0], -position[1], -position[2]));
  pcl::transformPointCloud(*transformed_cloud, *transformed_cloud, athena::transform::euler_matrix(0, 0, -box_geometry.yaw * M_PI / 180));
  pcl::transformPointCloud(*transformed_cloud, *transformed_cloud, athena::transform::translation_matrix(position[0], position[1], position[2]));
  athena::pointcloud::publishPointCloudXYZ(pub_transformed_cloud, *transformed_cloud, "world");
  pcl::getMinMax3D (*transformed_cloud, min_point_world, max_point_world);
  auto min_point = athena::conversions::toEigenVector3d(min_point_world);
  auto max_point = athena::conversions::toEigenVector3d(max_point_world);
  Eigen::Vector3d centre_diagonal = 0.5 * (min_point + max_point);
  box_geometry.OBB_dimensions = max_point - min_point;

  // if the filter_yaw parameter is set to true, it is verified whether the yaw found should be used. If the object is smaller than the set length and if their
  // x & y dimensions are within set length-width ratio (square) then the yaw is made 0. (Assumes that the object is symmetrical)
  if (req_params.filter_yaw) {
    if ((box_geometry.OBB_dimensions[0] < req_params.min_length) && (box_geometry.OBB_dimensions[0] / box_geometry.OBB_dimensions[1] > req_params.min_lw_ratio ||
         box_geometry.OBB_dimensions[0] / box_geometry.OBB_dimensions[1] < req_params.max_lw_ratio)) {
      box_geometry.yaw = 0;
    }
  }

  box_geometry.bounding_box = createVisualizationMarker(box_geometry.OBB_dimensions, centre_diagonal, box_geometry.yaw);

  return box_geometry;
}

double athena::pointcloud::computeBoundingBoxYaw(Eigen::Matrix3f rotation_matrix, Eigen::Vector3d position, Eigen::Vector3d AABB_dimensions, Eigen::Vector3d OBB_dimensions) {
  tf::TransformBroadcaster br_;
  std::vector<Eigen::Vector3d> world_point;

  // the box frame without translation is created to ease the yaw calculation
  Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
  projectionTransform.block<3,3>(0,0) = rotation_matrix;

  // three points in x, y & z-axis of the box frame is converted to points in world frame
  world_point = transformToWorldCoordinates(OBB_dimensions, projectionTransform.cast <double> ());

  // index sort consists of the rank of X, Y & Z-axis based on decreassing order of AABB dimensions.
  // For ex: if AABB dimesions are X - 0.1, Y - 0.05 & Z - 0.3, then index-sort looks like [1, 2, 0]
  std::vector<int> index_sort = sortAABBDimensions(AABB_dimensions);

  // bigger dimension of the box in the x-y plane of the world is chosen as the one that corresponds to world X-Axis
  // so that the canonical grasp condition is satisfied
  if (index_sort[0] > index_sort[1]) {
    int temp = index_sort[0];
    index_sort[0] = index_sort[1];
    index_sort[1] = temp;
  }

  // since the box frames have more than just yaw, it necessary to make one axis of the box (chosen based on sorting done) parallel to world-Z Axis
  // so that there is rotation only about world Z-axis.
  // to do the frame alignment, the initial angle difference between world Z-axis and the corresponding box-axis is calculated. the angle found is always
  // the shortest angle to world-Z axis
  double value = double(sqrt(pow(world_point[index_sort[2]].x(), 2) + pow(world_point[index_sort[2]].y(), 2)) / world_point[index_sort[2]].norm());
  double z_world = asin(value);

  // it is made sure that the box axis corresponding to world z-axis always point to the same direction as world Z irrespective of its initial orientation
  if (world_point[index_sort[2]].z() < 0) {
    if (index_sort[0] == 0) {
      projectionTransform *= athena::transform::euler_matrix(M_PI, 0, 0).cast <float> ();
    } else if (index_sort[0] == 1) {
      projectionTransform *= athena::transform::euler_matrix(0, M_PI, 0).cast <float> ();
    }
  }

  athena::transform::publish_matrix_as_tf(br_, projectionTransform.cast <double> (), "world", "obb_box_frame_without_translation");

  Eigen::Matrix4f possibleTransform1, possibleTransform2;
  std::vector<Eigen::Vector3d> point1, point2;
  double value1, value2;

  // since the box frame has both roll and pitch, it is necessary to rotate it back in the correct order to bring it closer to the desired orientation.
  // Hence, the rotation is performed in both ways and the one thats brings it closer to the desired orientation is chosen.

  // this rotates the box frame by angle difference computed above along that axis which corresponds to world-X axis.
  possibleTransform1 = rotateFrameAlongWorldX(OBB_dimensions, AABB_dimensions, projectionTransform.cast <double> (), z_world);
  point1 = transformToWorldCoordinates(OBB_dimensions, possibleTransform1.cast <double> ());
  value1 = double(sqrt(pow(point1[index_sort[2]].x(), 2) + pow(point1[index_sort[2]].y(), 2)) / point1[index_sort[2]].norm());

  // this rotates the box frame by angle difference computed above along that axis which corresponds to world-X axis.
  possibleTransform2 = rotateFrameAlongWorldY(OBB_dimensions, AABB_dimensions, projectionTransform.cast <double> (), z_world);
  point2 = transformToWorldCoordinates(OBB_dimensions, possibleTransform2.cast <double> ());
  value2 = double(sqrt(pow(point2[index_sort[2]].x(), 2) + pow(point2[index_sort[2]].y(), 2)) / point2[index_sort[2]].norm());

  // based on the resulting angle difference between world Z-axis and corresponding box axis after each rotation, the better one (the one)
  // with smaller angle difference after rotation) is chosen. This becomes the first rotation and the second rotation is done about the other
  // axis so that the final box frame is aligned with the world frame.
  if (value1 < value2) {
    projectionTransform = possibleTransform1;
    projectionTransform = rotateFrameAlongWorldY(OBB_dimensions, AABB_dimensions, projectionTransform.cast <double> (), asin(value1));
  } else {
    projectionTransform = possibleTransform2;
    projectionTransform = rotateFrameAlongWorldX(OBB_dimensions, AABB_dimensions, projectionTransform.cast <double> (), asin(value2));
  }

  athena::transform::publish_matrix_as_tf(br_, projectionTransform.cast <double> (), "world", "obb_box_corrected_frame_without_translation");

  world_point = transformToWorldCoordinates(OBB_dimensions, projectionTransform.cast <double> ());
  projectionTransform.block<3,1>(0,3) = position.cast <float> ();
  athena::transform::publish_matrix_as_tf(br_, projectionTransform.cast <double> (), "world", "obb_box_corrected_frame");

  // now that the frames are aligned (with just yaw rotation), yaw is calculated by finding the angle between world X-Axis
  // and the corresponding box axis.
  value = double(sqrt(pow(world_point[index_sort[0]].y(), 2) + pow(world_point[index_sort[0]].z(), 2)) / world_point[index_sort[0]].norm());

  if (world_point[index_sort[0]].x() * world_point[index_sort[0]].y() > 0) {
   return asin(value) * 180 / M_PI;
  }
  else {
   return -1 * asin(value) * 180 / M_PI;
  }
}

visualization_msgs::Marker athena::pointcloud::createVisualizationMarker(Eigen::Vector3d OBB_dimensions, Eigen::Vector3d center, double yaw) {
  auto orientation = athena::transform::euler_to_quaternion(0 ,0, yaw * M_PI / 180);

  visualization_msgs::Marker viz_geometry;
  viz_geometry.type = visualization_msgs::Marker::CUBE;
  viz_geometry.action = visualization_msgs::Marker::MODIFY;
  viz_geometry.header.stamp = ros::Time::now();
  viz_geometry.header.frame_id = "world";

  viz_geometry.scale.x = OBB_dimensions[0];
  viz_geometry.scale.y = OBB_dimensions[1];
  viz_geometry.scale.z = OBB_dimensions[2];

  viz_geometry.pose.position.x = center[0];
  viz_geometry.pose.position.y = center[1];
  viz_geometry.pose.position.z = center[2];
  viz_geometry.pose.orientation.x = orientation.x();
  viz_geometry.pose.orientation.y = orientation.y();
  viz_geometry.pose.orientation.z = orientation.z();
  viz_geometry.pose.orientation.w = orientation.w();

  viz_geometry.color.r = 1.0;
  viz_geometry.color.g = 0.0;
  viz_geometry.color.b = 0.0;
  viz_geometry.color.a = 0.475 + (0.05 *((double) rand() / (RAND_MAX)));

  return viz_geometry;
}

std::vector<int> athena::pointcloud::sortAABBDimensions(Eigen::Vector3d AABB_dimensions) {
  std::vector<int> sorted_index;
  if (AABB_dimensions.z() > AABB_dimensions.y()) {
    if (AABB_dimensions.z() > AABB_dimensions.x()) {
      if (AABB_dimensions.x() > AABB_dimensions.y()) {
        std::cout << "Z > X > Y" << "\n";
        sorted_index = {1, 2, 0};
      } else {
        std::cout << "Z > Y > X" << "\n";
        sorted_index = {2, 1, 0};
      }
    } else {
      std::cout << "X > Z > Y" << "\n";
      sorted_index = {0, 2, 1};
    }
  } else {
    if (AABB_dimensions.z() > AABB_dimensions.x()) {
      std::cout << "Y > Z > X" << "\n";
      sorted_index = {2, 0, 1};
    } else {
      if (AABB_dimensions.x() > AABB_dimensions.y()) {
        std::cout << "X > Y > Z" << "\n";
        sorted_index = {0, 1, 2};
      } else {
        std::cout << "Y > X > Z" << "\n";
        sorted_index = {1, 0, 2};
      }
    }
  }
  return sorted_index;
}

std::vector<Eigen::Vector3d> athena::pointcloud::transformToWorldCoordinates(Eigen::Vector3d OBB_dimensions, Eigen::Matrix4d transform) {
  std::vector<Eigen::Vector3d> world_point;
  Eigen::Vector3d pt1(0.0, 0.0, 0.0);
  Eigen::Vector3d pt2(0.0, 0.0, 0.0);
  Eigen::Vector3d pt3(0.0, 0.0, 0.0);

  pt1.x() = 0.5 * OBB_dimensions.x();
  pt2.y() = 0.5 * OBB_dimensions.y();
  pt3.z() = 0.5 * OBB_dimensions.z();

  world_point.push_back(athena::transform::transform_point(transform, pt1));
  world_point.push_back(athena::transform::transform_point(transform, pt2));
  world_point.push_back(athena::transform::transform_point(transform, pt3));

  return world_point;
}

Eigen::Matrix4f athena::pointcloud::rotateFrameAlongWorldX(Eigen::Vector3d OBB_dimensions, Eigen::Vector3d AABB_dimensions, Eigen::Matrix4d transform, double angle) {
  Eigen::Matrix4d transform1 = transform;
  auto world_point = transformToWorldCoordinates(OBB_dimensions, transform1);
  auto index_sort = sortAABBDimensions(AABB_dimensions);
  if ((world_point[index_sort[2]].y() > 0 && world_point[index_sort[0]].x() > 0) ||
      (world_point[index_sort[2]].y() < 0 && world_point[index_sort[0]].x() < 0)) {
    if (index_sort[0] == 0) {
      transform1 *= athena::transform::euler_matrix(angle, 0, 0);
    } else if (index_sort[0] == 1){
      transform1 *= athena::transform::euler_matrix(0, angle, 0);
    } else {
      transform1 *= athena::transform::euler_matrix(0, 0, angle);
    }
  } else if ((world_point[index_sort[2]].y() > 0 && world_point[index_sort[0]].x() < 0) ||
             (world_point[index_sort[2]].y() < 0 && world_point[index_sort[0]].x() > 0)) {
    if (index_sort[0] == 0) {
      transform1 *= athena::transform::euler_matrix(-angle, 0, 0);
    } else if (index_sort[0] == 1){
      transform1 *= athena::transform::euler_matrix(0, -angle, 0);
    } else {
      transform1 *= athena::transform::euler_matrix(0, 0, -angle);
    }
  }
  return transform1.cast <float> ();
}

Eigen::Matrix4f athena::pointcloud::rotateFrameAlongWorldY(Eigen::Vector3d OBB_dimensions, Eigen::Vector3d AABB_dimensions, Eigen::Matrix4d transform, double angle) {
  Eigen::Matrix4d transform1 = transform;
  auto world_point = transformToWorldCoordinates(OBB_dimensions, transform1);
  auto index_sort = sortAABBDimensions(AABB_dimensions);
  if ((world_point[index_sort[2]].x() > 0 && world_point[index_sort[1]].y() < 0) ||
      (world_point[index_sort[2]].x() < 0 && world_point[index_sort[1]].y() > 0)) {
    if (index_sort[1] == 0) {
      transform1 *= athena::transform::euler_matrix(angle, 0, 0);
    } else if (index_sort[1] == 1){
      transform1 *= athena::transform::euler_matrix(0, angle, 0);
    } else {
      transform1 *= athena::transform::euler_matrix(0, 0, angle);
    }
  } else if ((world_point[index_sort[2]].x() > 0 && world_point[index_sort[1]].y() > 0) ||
             (world_point[index_sort[2]].x() < 0 && world_point[index_sort[1]].y() < 0)) {
    if (index_sort[1] == 0) {
      transform1 *= athena::transform::euler_matrix(-angle, 0, 0);
    } else if (index_sort[1] == 1){
      transform1 *= athena::transform::euler_matrix(0, -angle, 0);
    } else {
      transform1 *= athena::transform::euler_matrix(0, 0, -angle);
    }
  }
  return transform1.cast <float> ();
}

Eigen::Vector3d athena::pointcloud::computePointCloudBoundingBoxOrigin(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  PointCloudProperties props = computePointCloudMinMax(cloud);
  Eigen::Vector3d result;
  result.x() = (props.max_point.x - props.min_point.x)/2.0 +  props.min_point.x;
  result.y() = (props.max_point.y - props.min_point.y)/2.0 +  props.min_point.y;
  result.z() = (props.max_point.z - props.min_point.z)/2.0 +  props.min_point.z;
  return result;
}
