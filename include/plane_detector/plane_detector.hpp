/*
 * Plane Detector Header
 * 
 * Defines the PlaneDetector class for detecting horizontal container surfaces
 * in LiDAR point clouds using RANSAC plane fitting.
 */



#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>    
#include <pcl/point_types.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>

class PlaneDetector : public rclcpp::Node
{
public:
  PlaneDetector();

private:
  // Node parameters
  std::string input_topic_;
  int n_points_threshold_;
  float sac_threshold_;
  std::string global_frame_;

  // Transform for converting detections to map frame
  geometry_msgs::msg::TransformStamped tf_2global_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // plane state
  pcl::PointIndices::Ptr plane_inliers_;
  bool detected_ = false;
  bool publish_plane_ = false;

  // Outlier removal parameters for cleaning detected planes
  int n_neighbors_;
  double radius_search_;

  // sub
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  // pub
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_hull_;
  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr pub_container_box_;

  void cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void detectPlanes(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                    pcl::PointIndices::Ptr &inliers, bool &detected_);
  void findCorners(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                   pcl::PointIndices::Ptr &inliers);
  void publishPlanePoints(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
      const pcl::PointIndices::Ptr &inliers,
      const std::string &frame_id);

  void publishPolygon(const geometry_msgs::msg::Polygon &polygon);
};
