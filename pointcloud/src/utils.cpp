#include "athena/pointcloud/utils.h"

// Computes the median of a point cloud
Eigen::Vector3d athena::pointcloud::computePointCloudMedian(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  std::vector<double> cloud_x, cloud_y, cloud_z;
  for (int i = 0; i < cloud->points.size(); i++){
    pcl::PointXYZ pt = cloud->points[i];
    cloud_x.push_back(pt.x);
    cloud_y.push_back(pt.y);
    cloud_z.push_back(pt.z);
    // std::cout << "  [athena] > point " << pt.x << " " << pt.y << " " << pt.z << "\n";
  }
  // std::cout << "  [athena] > point cloud size: " << cloud->points.size() << " (" << cloud_x.size() << " " << cloud_y.size() << " " << cloud_z.size() << ")\n";
  return computePointCloudMedian(cloud_x, cloud_y, cloud_z);
}

Eigen::Vector3d athena::pointcloud::computePointCloudMedian(std::vector<double> cluster_pt_x, std::vector<double> cluster_pt_y, std::vector<double> cluster_pt_z){

  sort(cluster_pt_x.begin(), cluster_pt_x.end());
  sort(cluster_pt_y.begin(), cluster_pt_y.end());
  sort(cluster_pt_z.begin(), cluster_pt_z.end());

  Eigen::Vector3d median( cluster_pt_x.at(cluster_pt_x.size()/2),
                          cluster_pt_y.at(cluster_pt_y.size()/2),
                          cluster_pt_z.at(cluster_pt_z.size()/2));
  // std::cout << "  [athena] > median computed: " << median.x() << " " << median.y() << " " << median.z() << "\n";
  return median;
}

Eigen::Vector3d athena::pointcloud::computePointCloudMedian(std::vector<Eigen::Vector3d> vec){
  std::vector<double> x, y, z;
  for (int i = 0; i < vec.size(); i++){
    x.push_back(vec.at(i).x());
    y.push_back(vec.at(i).y());
    z.push_back(vec.at(i).z());
  }
  return computePointCloudMedian(x, y, z);
}

std::vector<double> athena::pointcloud::getMinAndMaxFromVector(std::vector<double> my_vector){
  std::vector<double> min_max;
  int min_idx = std::min_element(my_vector.begin(), my_vector.end()) - my_vector.begin();
  int max_idx = std::max_element(my_vector.begin(), my_vector.end()) - my_vector.begin();

  min_max.push_back(my_vector.at(min_idx));
  min_max.push_back(my_vector.at(max_idx));
  return min_max;
}

// Helper function to directly publish a point cloud under the publisher with desired frame_id
void athena::pointcloud::publishPointCloudXYZ(ros::Publisher pub, pcl::PointCloud<pcl::PointXYZ> &pcl_cloud, std::string frame_id){
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::toPCLPointCloud2(pcl_cloud, pcl_pc2);
  sensor_msgs::PointCloud2 cloud_msg;
  pcl_conversions::fromPCL(pcl_pc2, cloud_msg);
  cloud_msg.header.frame_id = frame_id;
  pub.publish(cloud_msg);
}

void athena::pointcloud::publishPointCloudXYZRGB(ros::Publisher pub, pcl::PointCloud<pcl::PointXYZRGB> &pcl_cloud, std::string frame_id){
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::toPCLPointCloud2(pcl_cloud, pcl_pc2);
  sensor_msgs::PointCloud2 cloud_msg;
  pcl_conversions::fromPCL(pcl_pc2, cloud_msg);
  cloud_msg.header.frame_id = frame_id;
  pub.publish(cloud_msg);
}

Eigen::Vector3d athena::pointcloud::computePointCloudBoundingBoxOrigin(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  PointCloudProperties props = computePointCloudMinMax(cloud);
  Eigen::Vector3d result;
  result.x() = (props.max_point.x - props.min_point.x)/2.0 +  props.min_point.x;
  result.y() = (props.max_point.y - props.min_point.y)/2.0 +  props.min_point.y;
  result.z() = (props.max_point.z - props.min_point.z)/2.0 +  props.min_point.z;
  return result;
}

PointCloudProperties athena::pointcloud::computePointCloudMinMax(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  PointCloudProperties results;
  pcl::PointXYZ minPoint, maxPoint;
  pcl::getMinMax3D(*cloud, minPoint, maxPoint);
  results.min_point = minPoint;
  results.max_point = maxPoint;

  // std::cout << "Min Point: " << minPoint.x << " " << minPoint.y << " " << minPoint.z << "\n";
  // std::cout << "Max Point: " << maxPoint.x << " " << maxPoint.y << " " << maxPoint.z << "\n";
  return results;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr athena::pointcloud::getCloudPointsWithinStdDev(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, double thresh){
  pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (input_cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (thresh);
  sor.filter (*out_cloud);
  return out_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr athena::pointcloud::removeNegativeWorldPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, Eigen::Matrix4d cam_to_world){
  pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*input_cloud, *out_cloud, cam_to_world);
  out_cloud = athena::pointcloud::doPassThroughFilter(out_cloud, "z", 0.0, kInfinity);
  Eigen::Matrix4d world_to_cam = cam_to_world.inverse();   // Transform the cloud back into camera frame
  pcl::transformPointCloud(*out_cloud, *out_cloud, world_to_cam);
  return out_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr athena::pointcloud::doPassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string axis, double min, double max){
  pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName(axis);
  pass.setFilterLimits(min, max);
  pass.filter(*out_cloud);
  return out_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr athena::pointcloud::doPassThroughCubeCrop(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ pt, double radius){
  pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  out_cloud = athena::pointcloud::doPassThroughFilter(cloud, "x", pt.x-radius, pt.x+radius);
  out_cloud = athena::pointcloud::doPassThroughFilter(out_cloud, "y", pt.y-radius, pt.y+radius);
  out_cloud = athena::pointcloud::doPassThroughFilter(out_cloud, "z", pt.z-radius, pt.z+radius);
  return out_cloud;
}

pcl::PointIndices::Ptr athena::pointcloud::findCloudInliers(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ pt, double radius_x, double radius_y, double radius_z) {
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  for (int i = 0; i < cloud->points.size(); i++) {
    if (fabs(pt.x - cloud->points[i].x) < radius_x && fabs(pt.y - cloud->points[i].y) < radius_y && fabs(pt.z - cloud->points[i].z) < radius_z) {
      inliers->indices.push_back(i);
    }
  }
  return inliers;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr athena::pointcloud::removeSubCloudFromOriginalCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud_, pcl::PointXYZ pt,
                                       double radius_x, double radius_y, double radius_z) {
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  inliers = athena::pointcloud::findCloudInliers(original_cloud_, pt, radius_x, radius_y, radius_z);
  std::cout << "No.of inliers: " << inliers->indices.size() << "\n";

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(original_cloud_);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*original_cloud_);
  return original_cloud_;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr athena::pointcloud::getMaxEuclideanClusterFromPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, double tolerance){
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_extract = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  tree->setInputCloud (input_cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (tolerance); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (input_cloud);
  ec.extract (cluster_indices);

  if (cluster_indices.size() == 0 ){ return cloud_extract; }

  std::vector<int> cluster_sizes;
  for (int i = 0; i < cluster_indices.size(); i++){
    cluster_sizes.push_back(cluster_indices.at(i).indices.size());
  }
  auto result = std::max_element(cluster_sizes.begin(), cluster_sizes.end());
  int max_idx = std::distance(cluster_sizes.begin(), result);

  cloud_extract->width = cluster_indices.at(max_idx).indices.size();
  cloud_extract->height = 1;
  cloud_extract->is_dense = true;

  for (std::vector<int>::const_iterator pit = cluster_indices.at(max_idx).indices.begin (); pit != cluster_indices.at(max_idx).indices.end (); ++pit){
    cloud_extract->points.push_back (input_cloud->points[*pit]);
  }
  return cloud_extract;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr athena::pointcloud::createColorizedPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int r, int g, int b){
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;
  colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();

  colored_cloud->width = cloud->width;
  colored_cloud->height = cloud->height;
  colored_cloud->is_dense = true; //cloud->is_dense;

  std::vector<double> cloud_pts_x, cloud_pts_y, cloud_pts_z;
  for (size_t i_point = 0; i_point < cloud->points.size (); i_point++){
     pcl::PointXYZRGB point;
     point.x = *(cloud->points[i_point].data);
     point.y = *(cloud->points[i_point].data + 1);
     point.z = *(cloud->points[i_point].data + 2);
     point.r = r;
     point.g = g;
     point.b = b;

     cloud_pts_x.push_back(point.x);
     cloud_pts_y.push_back(point.y);
     cloud_pts_z.push_back(point.z);

     colored_cloud->points.push_back(point);
  }
  return colored_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr  athena::pointcloud::convertClusterToPointCloud(int idx, std::vector <pcl::PointIndices> clusters, pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_extract_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  auto cluster_idxs = clusters.at(idx).indices;
  cloud_extract_->is_dense = true;
  cloud_extract_->width = 0;
  cloud_extract_->height = 1;
  cloud_extract_->points.clear();
  for (int idx = 0; idx < cluster_idxs.size(); idx++){
    pcl::PointXYZ cloud_point = raw_cloud->at(cluster_idxs[idx]);
    cloud_extract_->points.push_back(cloud_point);
    cloud_extract_->width = cloud_extract_->width + 1;
  }
  return cloud_extract_;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr athena::pointcloud::doNeighborRadiusSearch(pcl::PointXYZ searchPoint, pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree_flann,
                                                                             pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud, double radius)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr extracted_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  extracted_cloud->width = 0;
  extracted_cloud->height = 1;
  extracted_cloud->is_dense = true;

  if (kd_tree_flann.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
      extracted_cloud->width = pointIdxRadiusSearch.size();
      extracted_cloud->points.clear();
      for (size_t i = 0; i < pointIdxRadiusSearch.size(); i++){
        extracted_cloud->points.push_back(raw_cloud->points[pointIdxRadiusSearch[i]]);
      }
   }
   //std::cout << "Extracted cloud size: " << extracted_cloud->points.size()  << "\n";
   return extracted_cloud;
}

BoundingBoxGeometry athena::pointcloud::obtainBoundingBoxGeomtry (pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, ros::Publisher pub_transformed_cloud) {
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

  auto obb_min_point = athena::pointcloud::toEigenVector3d(min_point_OBB);
  auto obb_max_point = athena::pointcloud::toEigenVector3d(max_point_OBB);
  auto aabb_min_point = athena::pointcloud::toEigenVector3d(min_point_AABB);
  auto aabb_max_point = athena::pointcloud::toEigenVector3d(max_point_AABB);
  auto position = athena::pointcloud::toEigenVector3d(position_OBB);

  box_geometry.AABB_dimensions = aabb_max_point - aabb_min_point;
  box_geometry.OBB_dimensions = obb_max_point - obb_min_point;

  Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
  projectionTransform.block<3,3>(0,0) = rotational_matrix_OBB;
  projectionTransform.block<3,1>(0,3) = position.cast <float> ();
  box_geometry.transformation_world_to_OBB.matrix() = projectionTransform.cast <double> ();

  box_geometry.yaw  = computeBoundingBoxYaw(rotational_matrix_OBB, position, box_geometry.AABB_dimensions, box_geometry.OBB_dimensions);

  pcl::transformPointCloud(*input_cloud, *transformed_cloud, athena::transform::translation_matrix(-position[0], -position[1], -position[2]));
  pcl::transformPointCloud(*transformed_cloud, *transformed_cloud, athena::transform::euler_matrix(0, 0, -box_geometry.yaw * M_PI / 180));
  pcl::transformPointCloud(*transformed_cloud, *transformed_cloud, athena::transform::translation_matrix(position[0], position[1], position[2]));
  publishPointCloudXYZ(pub_transformed_cloud, *transformed_cloud, "world");
  pcl::getMinMax3D (*transformed_cloud, min_point_world, max_point_world);
  auto min_point = athena::pointcloud::toEigenVector3d(min_point_world);
  auto max_point = athena::pointcloud::toEigenVector3d(max_point_world);
  box_geometry.OBB_dimensions = max_point - min_point;
  auto centre_diagonal = 0.5 * (min_point + max_point);

  box_geometry.bounding_box = createVisualizationMarker(box_geometry.OBB_dimensions, centre_diagonal, box_geometry.yaw);

  return box_geometry;
}

// step 0 : based on AABB dimension, find the OBB axis corresponding to x, y & z of the world
// step 1 : find angle b/w z-axis and corresponding OBB axis. If world z pt is +ve angle is angle, otherwise 180 - angle
// step 2 : if world pt has z-axis(x /y-axis) 0, then rotate it around OBB-axis corresponding to z-axis(x/y-axis).
// step 3 : based on AABB dimension choose the biggest in x-y axis and find the angle b/w that OBB axis and the world X-axis which is nothing but yaw
double athena::pointcloud::computeBoundingBoxYaw(Eigen::Matrix3f rotation_matrix, Eigen::Vector3d position, Eigen::Vector3d AABB_dimensions, Eigen::Vector3d OBB_dimensions) {
  tf2_ros::Buffer *tf_buffer_ = new tf2_ros::Buffer;
  tf2_ros::TransformListener *tf_listener_ = new tf2_ros::TransformListener(*tf_buffer_);
  tf::TransformBroadcaster br_;
  std::vector<Eigen::Vector3d> world_point;

  Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
  projectionTransform.block<3,3>(0,0) = rotation_matrix;
  athena::transform::publish_matrix_as_tf(br_, projectionTransform.cast <double> (), "world", "obb_box_frame_without_translation");

  world_point = transformToWorldCoordinates(*tf_buffer_ ,OBB_dimensions, "obb_box_frame_without_translation", projectionTransform.cast <double> ());

  std::vector<int> index_sort = sortAABBDimensions(AABB_dimensions);

  // choose the bigger dimension in x-y plane to compute yaw
  if (index_sort[0] > index_sort[1]) {
    int temp = index_sort[0];
    index_sort[0] = index_sort[1];
    index_sort[1] = temp;
  }

  double value = double(sqrt(pow(world_point[index_sort[2]].x(), 2) + pow(world_point[index_sort[2]].y(), 2)) / world_point[index_sort[2]].norm());
  double z_world = asin(value);
  if (world_point[index_sort[2]].z() < 0)
    z_world = M_PI - z_world ;

  if (world_point[index_sort[2]].y() > 0) {
    projectionTransform = athena::transform::euler_matrix(z_world, 0, 0).cast <float> () * projectionTransform;
  } else {
    projectionTransform = athena::transform::euler_matrix(-z_world, 0, 0).cast <float> () * projectionTransform;
  }

  /**
  if ((world_point[2].x() > 0 && z_world * 180 / M_PI < 90) || (world_point[2].x() < 0 && z_world * 180 / M_PI > 90)) {
    projectionTransform *= athena::transform::euler_matrix(z_world, 0, 0).cast <float> ();
  } else if ((world_point[2].x() < 0 && z_world * 180 / M_PI < 90) || (world_point[2].x() > 0 && z_world * 180 / M_PI > 90)){
    projectionTransform *= athena::transform::euler_matrix(-z_world, 0, 0).cast <float> ();
  }
  **/
  athena::transform::publish_matrix_as_tf(br_, projectionTransform.cast <double> (), "world", "obb_box_corrected_frame_without_translation");

  world_point = transformToWorldCoordinates(*tf_buffer_, OBB_dimensions, "obb_box_corrected_frame_without_translation", projectionTransform.cast <double> ());
  projectionTransform.block<3,1>(0,3) = position.cast <float> ();
  athena::transform::publish_matrix_as_tf(br_, projectionTransform.cast <double> (), "world", "obb_box_corrected_frame");

  value = double(sqrt(pow(world_point[index_sort[0]].y(), 2) + pow(world_point[index_sort[0]].z(), 2)) / world_point[index_sort[0]].norm());
  if (world_point[index_sort[0]].x() * world_point[index_sort[0]].y() > 0)
   return asin(value) * 180 / M_PI;
  else
   return -1 * asin(value) * 180 / M_PI;
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

std::vector<Eigen::Vector3d> athena::pointcloud::transformToWorldCoordinates(tf2_ros::Buffer &tf_buffer, Eigen::Vector3d OBB_dimensions, std::string source_frame, Eigen::Matrix4d transform) {
  std::vector<Eigen::Vector3d> world_point, calc_point;
  Eigen::Vector3d pt1, pt2, pt3;
  geometry_msgs::PoseStamped p1, p2, p3;
  p1.pose.position.x = 0.5 * OBB_dimensions.x();
  p2.pose.position.y = 0.5 * OBB_dimensions.y();
  p3.pose.position.z = 0.5 * OBB_dimensions.z();
  pt1.x() = 0.5 * OBB_dimensions.x();
  pt1.y() = 0;
  pt1.z() = 0;
  pt2.y() = 0.5 * OBB_dimensions.y();
  pt2.x() = 0;
  pt2.z() = 0;
  pt3.z() = 0.5 * OBB_dimensions.z();
  pt3.y() = 0;
  pt3.x() = 0;

  calc_point.push_back(athena::transform::transform_point(transform, pt1));
  calc_point.push_back(athena::transform::transform_point(transform, pt2));
  calc_point.push_back(athena::transform::transform_point(transform, pt3));

  p1 = athena::transform::transform_point(tf_buffer, p1, source_frame, "world");
  p2 = athena::transform::transform_point(tf_buffer, p2, source_frame, "world");
  p3 = athena::transform::transform_point(tf_buffer, p3, source_frame, "world");

  world_point.push_back(toEigenVector3d(p1));
  world_point.push_back(toEigenVector3d(p2));
  world_point.push_back(toEigenVector3d(p3));

  Eigen::Affine3d box_to_world_ = tf2::transformToEigen(tf_buffer.lookupTransform(source_frame, "world", ros::Time(0)));

  std::cout << "Tf lookupTransform \n";
  std::cout << box_to_world_.matrix() << "\n";
  std::cout << "Transform inverse \n";
  std::cout << transform << "\n";
  std::cout << "Transform \n";
  std::cout << transform.inverse() << "\n";

 /**
  std::cout << "Point 1 using tf : " << world_point[0].x() << " " << world_point[0].y() << " " << world_point[0].z() << "\n";
  std::cout << "Point 1 without tf :" << calc_point[0].x() << " " << calc_point[0].y() << " " << calc_point[0].z() << "\n";
  std::cout << "Point 2 using tf : " << world_point[1].x() << " " << world_point[1].y() << " " << world_point[1].z() << "\n";
  std::cout << "Point 2 without tf :" << calc_point[1].x() << " " << calc_point[1].y() << " " << calc_point[1].z() << "\n";
  std::cout << "Point 3 using tf : " << world_point[2].x() << " " << world_point[2].y() << " " << world_point[2].z() << "\n";
  std::cout << "Point 3 without tf :" << calc_point[2].x() << " " << calc_point[2].y() << " " << calc_point[2].z() << "\n";
 **/
  return calc_point;
}
