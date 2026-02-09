#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <container_perception/msg/polygon_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <mutex>
#include <vector>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>


struct Leg {
    int container_id; // ID of the container this leg belongs to
    int leg_id;       // ID of the leg within the container -> 0 to 3, based on polygon point order
    float angle;    // Angle of the leg w.r.t the vertical axis!

};


class LegDetector : public rclcpp::Node
{
public:
    LegDetector();

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_d435_;
    rclcpp::Subscription<container_perception::msg::PolygonArray>::SharedPtr sub_containers_;
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_detected_leg_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cropbox_marker_pub_;

    void callback_d435(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void callback_containers(const container_perception::msg::PolygonArray & msg);

    void setCropBox(const geometry_msgs::msg::Polygon& poly, Eigen::Vector4f& crop_min, Eigen::Vector4f& crop_max);
    

    void publishLeg(const sensor_msgs::msg::PointCloud2& msg);

    void publishCropBoxMarker(const Eigen::Vector4f& crop_min,
                              const Eigen::Vector4f& crop_max,
                              const std::string& frame_id,
                              int id);

    void findLegId(const geometry_msgs::msg::Polygon& poly, Eigen::Vector4f centroid, int& leg_id);
    void computeLegAngle(pcl::PointCloud<pcl::PointXYZRGB>::Ptr leg_cloud, float& angle, int leg_id, int container_id);

    
    rclcpp::CallbackGroup::SharedPtr cb_group;
    rclcpp::SubscriptionOptions sub_options;
    // TF
    geometry_msgs::msg::TransformStamped tf_2_sensor_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    //Parameters
    int n_points_threshold_;
    bool publish_in_rvizz_;
    float delta_crop_;
    container_perception::msg::PolygonArray tracked_containers_;
    std::mutex containers_mtx_;
    std::string global_frame_;
    std::string sensor_frame_;

    std::vector<Leg> tracked_legs_;

    //?void detectLegsInCloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);

    
};

