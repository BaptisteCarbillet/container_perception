#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <container_perception/msg/polygon_array.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/centroid.h>
#include <geometry_msgs/msg/polygon.hpp>
#include <container_perception/srv/localize_legs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <boost/optional.hpp>
//include boost
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>


#include <mutex>
#include <limits>

struct ClusterResult {
  std::vector<pcl::PointIndices> clusters;
  std::vector<Eigen::Vector4f> centroids;
};

class LegLocalizer : public rclcpp::Node
{
public:
  LegLocalizer();
private:
  // Subscriptions
    rclcpp::Subscription<container_perception::msg::PolygonArray>::SharedPtr sub_containers_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    // Publications
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
    
    // Service
    rclcpp::Service<container_perception::srv::LocalizeLegs>::SharedPtr srv_;
    // Cached state
    container_perception::msg::PolygonArray containers_;
    sensor_msgs::msg::PointCloud2::SharedPtr cloud_cache_;
    std::mutex containers_mtx_;
    std::mutex cloud_mtx_;
    void cbContainers(const container_perception::msg::PolygonArray::SharedPtr msg);
    void cbCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void cbService(
    const std::shared_ptr<container_perception::srv::LocalizeLegs::Request> req,
    std::shared_ptr<container_perception::srv::LocalizeLegs::Response> res);

    void correctingClusters(ClusterResult& leg_clusters, pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud);

    void setCropBox(const geometry_msgs::msg::Polygon& poly, Eigen::Vector4f& crop_min, Eigen::Vector4f& crop_max);
    void clusteringCroppedCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud, ClusterResult& leg_clusters);
    void publishCentroidsMarkers(const std::vector<Eigen::Vector4f>& centroids);
    float delta_crop_;
    float cluster_tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;

  };


