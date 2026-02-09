
/*
 * Plane Detector Node
 * 
 * Detects horizontal planes at a fixed height in LiDAR point clouds to identify container surfaces.
 * Uses RANSAC segmentation to find planes, then computes minimum area rectangles representing
 * container boundaries. Results are published in the map frame for tracking.
 * 
 * Subscriptions:
 *   - /livox/lidar_cropped (PointCloud2): Cropped LiDAR data from cropbox node
 * 
 * Publications:
 *   - /plane_detector/container_box (Polygon): Detected container boundary (4 corners in map frame)
 *   - /plane_detector/plane_points (PointCloud2): Visualization of detected plane (if publish_plane=true)
 *   - /plane_detector/plane_hull (PointCloud2): Visualization of convex hull of detected plane (if publish_plane=true)
 * 
 * Key Parameters:
 *   - n_points_threshold: Minimum points required to trigger detection
 *   - sac_threshold: RANSAC distance threshold for plane fitting
 *   - radius_search/n_neighbors: Outlier removal parameters
 */


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/point_types.h>
#include <unordered_set>
#include <pcl/PointIndices.h>
#include <pcl/common/centroid.h>
#include "plane_detector/plane_detector.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <opencv2/opencv.hpp>
#include <vector>


PlaneDetector::PlaneDetector()
: Node("plane_detector"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  input_topic_        = this->declare_parameter<std::string>("input_topic", "/livox/lidar_cropped");
  n_points_threshold_ = this->declare_parameter<int>("n_points_threshold", 13000);
  sac_threshold_      = this->declare_parameter<double>("sac_threshold", 0.02);
  global_frame_       = this->declare_parameter<std::string>("global_frame", "map");

  plane_inliers_ = std::make_shared<pcl::PointIndices>();
  publish_plane_ = this->declare_parameter<bool>("publish_plane", false);

  n_neighbors_ = this->declare_parameter<int>("n_neighbors", 10);
  radius_search_ = this->declare_parameter<double>("radius_search", 0.1);

  sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&PlaneDetector::cb, this, std::placeholders::_1)
  );

  pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/plane_detector/plane_points",
    rclcpp::SensorDataQoS()
  );

  pub_hull_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/plane_detector/plane_hull",
    rclcpp::SensorDataQoS()
  );

  pub_container_box_ = this->create_publisher<geometry_msgs::msg::Polygon>(
    "/plane_detector/container_box",
    rclcpp::SensorDataQoS()
  );

  
}

/**
 * @brief Main callback for processing incoming point cloud data
 * 
 * Pipeline: Check point count -> Transform to map frame -> Downsample -> 
 * Detect plane with RANSAC -> Find corners -> Publish container boundary
 */

void PlaneDetector::cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (msg->height * msg->width < static_cast<std::size_t>(n_points_threshold_)) {
    return;
  }
  
  // Lookup transform to map frame for consistent container tracking across robot motion
  try {
    tf_2global_ = tf_buffer_.lookupTransform(
      global_frame_,
      msg->header.frame_id,
      msg->header.stamp,
      tf2::durationFromSec(0.3));
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s",
                msg->header.frame_id.c_str(), global_frame_.c_str(), ex.what());
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr in(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *in);

  // Downsample to reduce computation while maintaining plane detection accuracy (5cm voxels)
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setInputCloud(in);
  voxel_grid.setLeafSize(0.05f, 0.05f, 0.05f);
  voxel_grid.filter(*downsampled);

  plane_inliers_->indices.clear();
  detected_ = false;
  detectPlanes(downsampled, plane_inliers_, detected_);

  if (detected_) {

    if (publish_plane_) {
      publishPlanePoints(downsampled, plane_inliers_, msg->header.frame_id);
    }
    findCorners(downsampled, plane_inliers_);
  }

}

/**
 * @brief Detect horizontal planes using RANSAC segmentation
 * 
 * @param cloud Input downsampled point cloud
 * @param inliers Output indices of points belonging to detected plane
 * @param detected_ Output flag indicating if a valid plane was found
 */
void PlaneDetector::detectPlanes(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                 pcl::PointIndices::Ptr &inliers, bool &detected_)
{
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(sac_threshold_);

  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.empty()) {
    RCLCPP_WARN(this->get_logger(),
                "Could not estimate a planar model for the given dataset.");
    detected_ = false;
    return;
  }

  //RCLCPP_INFO(this->get_logger(),
  //            "Plane detected with %zu inliers.", inliers->indices.size());
  detected_ = true;
}



/**
 * @brief Extract container boundary from detected plane
 * 
 * Process: Extract plane points -> Remove outliers -> Compute 2D convex hull ->
 * Fit minimum area rectangle -> Transform to map frame -> Publish polygon
 * 
 * @param cloud Input cloud
 * @param inliers Indices of points in the detected plane
 */
void PlaneDetector::findCorners(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                pcl::PointIndices::Ptr &inliers)
{


  //Filter the plane points
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*plane_cloud);

  // Remove isolated outlier points that don't belong to the main container surface
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
  outrem.setInputCloud(plane_cloud);
  outrem.setRadiusSearch(radius_search_);
  outrem.setMinNeighborsInRadius(n_neighbors_);
  outrem.filter (*filtered_plane_cloud);



  // Compute 2D convex hull of the plane (assumes flat horizontal container surfaces) 
  pcl::PointCloud<pcl::PointXYZ>::Ptr hull(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ConvexHull<pcl::PointXYZ> hull_computer;
  hull_computer.setInputCloud(filtered_plane_cloud);
  hull_computer.setDimension(2);
  hull_computer.setComputeAreaVolume(false);
  hull_computer.reconstruct(*hull);

  //RCLCPP_INFO(this->get_logger(),
  //            "Convex hull has %zu points.", hull->points.size());

    if (publish_plane_) {
      sensor_msgs::msg::PointCloud2 hull_msg;
      pcl::toROSMsg(*hull, hull_msg);
      hull_msg.header.frame_id = cloud->header.frame_id;
      hull_msg.header.stamp = this->now();
      pub_hull_->publish(hull_msg);
    }


    // Fit minimum area rectangle to hull points to get container boundary
    std::vector<cv::Point2f> points_in_hull;
    for (const auto& point : hull->points) {
      points_in_hull.push_back(cv::Point2f(point.x, point.y));
    }
    cv::RotatedRect min_rect = cv::minAreaRect(points_in_hull);
    cv::Point2f rect_points[4];
    min_rect.points(rect_points);

    /*RCLCPP_INFO(this->get_logger(),
                "Minimum area rectangle corners:");
    for (int i = 0; i < 4; ++i) {
      RCLCPP_INFO(this->get_logger(),
                  "Corner %d: (%.2f, %.2f)", i, rect_points[i].x, rect_points[i].y);
    }*/

    // Use mean Z height of hull as container height (assumes flat horizontal planes)
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*hull,centroid);
    float mean_z = centroid[2];
    
    // Build polygon message with 4 corners and transform to map frame for tracking
    
    geometry_msgs::msg::Polygon polygon_msg;
    for (int i = 0; i < 4; ++i) {
      geometry_msgs::msg::Point32 pt;
      pt.x = rect_points[i].x;
      pt.y = rect_points[i].y;
      pt.z = mean_z; // Flat plane assumption: all corners at same height
      polygon_msg.points.push_back(pt);

    }
    geometry_msgs::msg::Polygon polygon_in_global;
    tf2::doTransform(polygon_msg, polygon_in_global, tf_2global_);

    publishPolygon(polygon_in_global);

  }

void PlaneDetector::publishPolygon(const geometry_msgs::msg::Polygon &polygon)
{
    pub_container_box_->publish(polygon);
}



/**
 * @brief Publish colored visualization of detected plane for RViz debugging
 * 
 * Inlier points (kept after outlier removal) are colored red, removed outliers are blue.
 * Only published when publish_plane parameter is true.
 */
void PlaneDetector::publishPlanePoints(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
  const pcl::PointIndices::Ptr &inliers,
  const std::string &frame_id)

{
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*plane_cloud);

  // Apply same outlier removal to identify which points to color red vs blue
  pcl::Indices kept_indices;

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
  outrem.setInputCloud(plane_cloud);
  outrem.setRadiusSearch(radius_search_);
  outrem.setMinNeighborsInRadius(n_neighbors_);
  outrem.filter(kept_indices);

  std::vector<uint8_t> is_kept(plane_cloud->size(), 0);
  for (int idx : kept_indices) {
    if (idx>=0 && static_cast<size_t>(idx) < is_kept.size()) {
      is_kept[idx] = 1;
      
    }
  }

  

  

  // Create RGB point cloud: red = kept inliers, blue = removed outliers
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
  plane_cloud_rgb->resize(plane_cloud->size());

  for (size_t i = 0; i < plane_cloud->points.size(); ++i) {
    pcl::PointXYZRGB point_rgb;
    point_rgb.x = plane_cloud->points[i].x;
    point_rgb.y = plane_cloud->points[i].y;
    point_rgb.z = plane_cloud->points[i].z;

    // Check if the point was removed
    if (is_kept[i]) {
      // Inlier point (part of container surface) - red
      point_rgb.r = 255;
      point_rgb.g = 0;
      point_rgb.b = 0;
    } else {
      // Outlier point (removed) - blue
      point_rgb.r = 0;
      point_rgb.g = 0;
      point_rgb.b = 255;
    }

    plane_cloud_rgb->points[i] = point_rgb;
  }
  

  


  sensor_msgs::msg::PointCloud2 out_msg;
  pcl::toROSMsg(*plane_cloud_rgb, out_msg);
  out_msg.header.frame_id = frame_id;
  out_msg.header.stamp = this->now();
  pub_->publish(out_msg);
}