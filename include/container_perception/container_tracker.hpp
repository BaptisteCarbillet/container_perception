/*
 * Container Tracker Header
 * 
 * Defines the ContainerTracker class for maintaining persistent tracking
 * of detected containers across multiple detections.
 */



#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <std_msgs/msg/int32.hpp>
#include <visualization_msgs/msg/marker_array.hpp> 
#include <container_perception/msg/polygon_array.hpp>

struct Container {
        cv::RotatedRect box;  // 2D oriented bounding box in map frame
        float height;         // Z coordinate in map frame
        int id;               // Unique persistent identifier
};

class ContainerTracker : public rclcpp::Node
{
public:
  ContainerTracker();

private:
    
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr sub_container_box_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    
    // Publishers
    rclcpp::Publisher<container_perception::msg::PolygonArray>::SharedPtr containers_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr id_publisher_;
    int next_container_id_ = 0;
    std::vector<Container> tracked_containers_; //List of all tracked containers

    void cbContainerBox(const geometry_msgs::msg::Polygon::SharedPtr msg);
    bool overlap(const Container& c1, const Container& c2);
    void fuse(Container& c1, Container& c2);
    void insert(Container& new_container);
    void publishMarkers();
    void publishContainers();
};

