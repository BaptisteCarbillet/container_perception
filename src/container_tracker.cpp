#include <container_perception/container_tracker.hpp>

/*
 * Container Tracker Node
 * 
 * Maintains a persistent list of detected containers by fusing overlapping detections
 * from the plane_detector node. Assigns unique IDs to newly detected containers and
 * publishes tracking results in the map frame.
 * 
 * Subscriptions:
 *   - /plane_detector/container_box (Polygon): Detected container boundaries
 * 
 * Publications:
 *   - container_tracker/containers (PolygonArray): All tracked containers with IDs
 *   - container_tracker/markers (MarkerArray): RViz visualization markers
 *   - container_tracker/new_id_detected (Int32): Notification when new container found
 */

ContainerTracker::ContainerTracker()
: Node("container_tracker")
{
    std::string container_box_topic = this->declare_parameter<std::string>(
        "container_box_topic", "/plane_detector/container_box");

    sub_container_box_ = this->create_subscription<geometry_msgs::msg::Polygon>(
        container_box_topic, rclcpp::SensorDataQoS(), std::bind(&ContainerTracker::cbContainerBox, this, std::placeholders::_1));


    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "container_tracker/markers", rclcpp::SensorDataQoS());
    // publish Polygon array msg
    containers_pub_ = this->create_publisher<container_perception::msg::PolygonArray>(
        "container_tracker/containers", rclcpp::SensorDataQoS());


    id_publisher_ = this->create_publisher<std_msgs::msg::Int32>(
        "container_tracker/new_id_detected", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());


    //Use_sim_time parameters
    if (!this->has_parameter("use_sim_time")) {
        this->declare_parameter<bool>("use_sim_time", false);
        }
    
    bool use_sim_time = this->get_parameter("use_sim_time").as_bool();
    if (use_sim_time) {
        RCLCPP_INFO(this->get_logger(), "Using simulation time");
    } else {
        RCLCPP_INFO(this->get_logger(), "Using system time");
    }
}

/**
 * @brief Callback for incoming container detections from plane_detector
 * 
 * Converts Polygon message to cv::RotatedRect, creates Container object,
 * and attempts insertion into tracked list (with automatic fusion if overlapping).
 */
void ContainerTracker::cbContainerBox(const geometry_msgs::msg::Polygon::SharedPtr msg)
{
    // Convert Polygon points to cv::Point2f
    std::vector<cv::Point2f> box_points;
    for (const auto& point : msg->points) {
        box_points.push_back(cv::Point2f(point.x, point.y));
    }

    // Fit a rotated rectangle to the points
    cv::RotatedRect box = cv::minAreaRect(box_points);

    // Store the tracked container
    Container container;
    container.box = box;
    container.height = msg->points[0].z;
    insert(container);

    //std::cout << "Tracked Containers: " << tracked_containers_.size() << std::endl;
    //RCLCPP_INFO(this->get_logger(), "Tracked Containers : %d", static_cast<int>(tracked_containers_.size()));

}

/**
 * @brief Check if two containers overlap using OpenCV rotated rectangle intersection
 * 
 * @return true if containers have any spatial overlap (partial or full)
 */
bool ContainerTracker::overlap(const Container& c1, const Container& c2)
{
    // Check if two rotated rectangles overlap using OpenCV function
    std::vector<cv::Point2f> intersectingRegion;
    int result = cv::rotatedRectangleIntersection(c1.box, c2.box, intersectingRegion);

    return (result == cv::INTERSECT_FULL || result == cv::INTERSECT_PARTIAL);
}

/**
 * @brief Fuse two overlapping containers into one
 * 
 * Combines all 8 corner points from both containers and fits a new minimum area
 * rectangle. Result is stored in c1, effectively merging c2 into c1.
 */
void ContainerTracker::fuse(Container& c1, Container& c2)
{
    std::vector<cv::Point2f> points;
    points.reserve(8);
    cv::Point2f vertices[4];
    c1.box.points(vertices);
    for (int i = 0; i < 4; ++i) {
        points.push_back(vertices[i]);
    }
    
    c2.box.points(vertices);
    for (int i = 0; i < 4; ++i) {
        points.push_back(vertices[i]);
    }

    // Fit a new rotated rectangle to the combined points
    cv::RotatedRect fused_box = cv::minAreaRect(points);
    c1.box = fused_box;
    
}

/**
 * @brief Insert new container detection into tracked list
 * 
 * If new detection overlaps with existing container, they are fused.
 * Otherwise, assigns new unique ID and adds to tracking list.
 */
void ContainerTracker::insert(Container& new_container)
{
    // Check for overlap with existing containers
    for (auto& tracked_container : tracked_containers_) {
        if (overlap(tracked_container, new_container)) {
            fuse(tracked_container, new_container);
            publishMarkers();
            publishContainers();
            return;
        }
    }

    // If no overlap, add as a new tracked container
    new_container.id = next_container_id_++;
    tracked_containers_.push_back(new_container);
    // Publish new ID
    std_msgs::msg::Int32 id_msg;
    id_msg.data = new_container.id;
    id_publisher_->publish(id_msg);
    RCLCPP_INFO(this->get_logger(), "Tracked Containers : %d", static_cast<int>(tracked_containers_.size()));
    publishMarkers();
    publishContainers();
}

/**
 * @brief Publish all tracked containers as PolygonArray with associated IDs
 */
void ContainerTracker::publishContainers()
{
    container_perception::msg::PolygonArray container_array_msg;
    

    for (const auto& container : tracked_containers_) {
        geometry_msgs::msg::Polygon polygon_msg;
        cv::Point2f vertices[4];
        container.box.points(vertices);
        for (int i = 0; i < 4; ++i) {
            geometry_msgs::msg::Point32 pt;
            pt.x = vertices[i].x;
            pt.y = vertices[i].y;
            pt.z = container.height; // TODO : Handle height later
            polygon_msg.points.push_back(pt);
        }
        container_array_msg.polygons.push_back(polygon_msg);
        container_array_msg.ids.push_back(container.id);
    }

    containers_pub_->publish(container_array_msg);
}

/**
 * @brief Publish RViz LINE_STRIP markers for visualization of tracked containers
 */
void ContainerTracker::publishMarkers()
{
    visualization_msgs::msg::MarkerArray marker_array;
    for (const auto& container : tracked_containers_) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map"; // Assuming global frame is "map"
        marker.header.stamp = this->now();
        marker.ns = "containers";
        marker.id = container.id;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.05; // Line width
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;

        cv::Point2f vertices[4];
        container.box.points(vertices);
        for (int i = 0; i < 4; ++i) {
            geometry_msgs::msg::Point p;
            p.x = vertices[i].x;
            p.y = vertices[i].y;
            p.z = container.height; // TODO : Handle height later
            marker.points.push_back(p);
        }
        // Close the rectangle
        geometry_msgs::msg::Point p;
        p.x = vertices[0].x;
        p.y = vertices[0].y;
        p.z = container.height;
        marker.points.push_back(p);

        marker_array.markers.push_back(marker);
    }

    marker_pub_->publish(marker_array);
}