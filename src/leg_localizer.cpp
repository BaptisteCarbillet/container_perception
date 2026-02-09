#include <container_perception/leg_localizer.hpp>

LegLocalizer::LegLocalizer()
: Node("leg_localizer")

{


    sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud_map", rclcpp::SensorDataQoS(),
        std::bind(&LegLocalizer::cbCloud, this, std::placeholders::_1));

    sub_containers_ = this->create_subscription<container_perception::msg::PolygonArray>(
        "/container_tracker/containers", rclcpp::SensorDataQoS(),
        std::bind(&LegLocalizer::cbContainers, this, std::placeholders::_1));

    srv_ = this->create_service<container_perception::srv::LocalizeLegs>(
        "localize_legs",
        std::bind(&LegLocalizer::cbService, this,
                  std::placeholders::_1, std::placeholders::_2));

    delta_crop_ = this->declare_parameter<float>("delta_crop",0.0);
    cluster_tolerance_ = this->declare_parameter<float>("cluster_tolerance",0.05);
    min_cluster_size_ = this->declare_parameter<int>("min_cluster_size",10);
    max_cluster_size_ = this->declare_parameter<int>("max_cluster_size",500);  
    
    pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "leg_localizer/centroids",
        rclcpp::SensorDataQoS());

    RCLCPP_INFO(this->get_logger(), "LegLocalizer node has been initialized.");
    //print parameters
    RCLCPP_INFO(this->get_logger(), "Parameters: delta_crop=%f, cluster_tolerance=%f, min_cluster_size=%d, max_cluster_size=%d",
        delta_crop_, cluster_tolerance_, min_cluster_size_, max_cluster_size_);
}


void LegLocalizer::cbContainers(const container_perception::msg::PolygonArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(containers_mtx_);
    containers_ = *msg;
}

void LegLocalizer::cbCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(cloud_mtx_);
    cloud_cache_ = msg;
}

void LegLocalizer::setCropBox(const geometry_msgs::msg::Polygon& poly, Eigen::Vector4f& crop_min, Eigen::Vector4f& crop_max) {
    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float min_z = -10.0f;//std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float max_y = std::numeric_limits<float>::lowest();
    float max_z = 10.0f;//std::numeric_limits<float>::lowest();
    for (const auto & p : poly.points) {
        if (p.x < min_x) min_x = p.x;
        if (p.y < min_y) min_y = p.y;
        //if (p.z < min_z) min_z = p.z;

        if (p.x > max_x) max_x = p.x;
        if (p.y > max_y) max_y = p.y;
        //if (p.z > max_z) max_z = p.z;
    }
    crop_min = Eigen::Vector4f(min_x - delta_crop_, min_y - delta_crop_, min_z, 1.0f);
    crop_max = Eigen::Vector4f(max_x + delta_crop_, max_y + delta_crop_, max_z, 1.0f);
}


void LegLocalizer::correctingClusters(ClusterResult& leg_clusters, pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud) {
    //If there is less than 4 clusters, we do nothing, otherwise we merge the closest clusters until we have 4
    while (leg_clusters.clusters.size() > 4) {
        float min_distance = std::numeric_limits<float>::max();
        size_t idx1 = 0;
        size_t idx2 = 0;
        for (size_t i = 0; i < leg_clusters.centroids.size(); ++i) {
            for (size_t j = i + 1; j < leg_clusters.centroids.size(); ++j) {
                float distance = (leg_clusters.centroids[i] - leg_clusters.centroids[j]).norm();
                if (distance < min_distance) {
                    min_distance = distance;
                    idx1 = i;
                    idx2 = j;
        
                }
            }
        }

        // Ensure idx1 < idx2 for safe removal
        if (idx2 < idx1) {
            std::swap(idx1, idx2);
        }

        // Merge clusters idx1 and idx2
        leg_clusters.clusters[idx1].indices.insert(
            leg_clusters.clusters[idx1].indices.end(),
            leg_clusters.clusters[idx2].indices.begin(),
            leg_clusters.clusters[idx2].indices.end()
        );

        //Delete cluster idx2
        leg_clusters.clusters.erase(leg_clusters.clusters.begin() + idx2);

        // Recompute centroid for merged cluster
        Eigen::Vector4f new_centroid;
        pcl::compute3DCentroid(*cropped_cloud, leg_clusters.clusters[idx1], new_centroid);
        leg_clusters.centroids[idx1] = new_centroid;
        
    }
}

void LegLocalizer::clusteringCroppedCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud, ClusterResult& leg_clusters) {
    // Implement Euclidean clustering to find leg clusters in cropped_cloud
    // Fill leg_clusters with the resulting clusters
    ClusterResult result;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cropped_cloud);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance_); // 0.05 == 5cm
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cropped_cloud);
    ec.extract(leg_clusters.clusters);

    // Compute centroids for each cluster
    for (const auto& indices : leg_clusters.clusters) {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cropped_cloud, indices, centroid);
        leg_clusters.centroids.push_back(centroid);
    }
}

void LegLocalizer::cbService(
    const std::shared_ptr<container_perception::srv::LocalizeLegs::Request> req,
    std::shared_ptr<container_perception::srv::LocalizeLegs::Response> res) 
{
    RCLCPP_INFO(get_logger(), "Service called for container_id=%d", req->container_id);

    // Lock the containers and cloud
    container_perception::msg::PolygonArray containers_copy;
    sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg;
    {
        std::lock_guard<std::mutex> lk1(containers_mtx_);
        containers_copy = containers_;
    }
    {
        std::lock_guard<std::mutex> lk2(cloud_mtx_);
        cloud_msg = cloud_cache_;
    }

    if (!cloud_msg) {
        res->success = false;
        res->message = "No cloud data available";
        RCLCPP_WARN(get_logger(), "Service failed: %s", res->message.c_str());
        return;
    }

    if (containers_copy.polygons.empty()) {
        res->success = false;
        res->message = "No containers available";
        RCLCPP_WARN(get_logger(), "Service failed: %s", res->message.c_str());
        return;
    }

    // Find the container with matching ID
    int target_index = -1;
    for (size_t i = 0; i < containers_copy.ids.size(); ++i) {
        if (containers_copy.ids[i] == req->container_id) {
            target_index = static_cast<int>(i);
            break;
        }
    }

    if (target_index < 0 || target_index >= static_cast<int>(containers_copy.polygons.size())) {
        res->success = false;
        res->message = "Container ID " + std::to_string(req->container_id) + " not found";
        RCLCPP_WARN(get_logger(), "Service failed: %s", res->message.c_str());
        return;
    }

    RCLCPP_INFO(get_logger(), "Found container at index %d, converting cloud...", target_index);

    // Convert only when needed
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_copy(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud_msg, *cloud_copy);

    RCLCPP_INFO(get_logger(), "Cloud converted, %zu points. Cropping...", cloud_copy->size());

    // Process the cloud and containers to localize legs
    geometry_msgs::msg::Polygon target_container;
    target_container = containers_copy.polygons[target_index];

    Eigen::Vector4f crop_min, crop_max;
    setCropBox(target_container, crop_min, crop_max);
    pcl::CropBox<pcl::PointXYZ> crop_filter;
    crop_filter.setMin(crop_min);
    crop_filter.setMax(crop_max);
    crop_filter.setInputCloud(cloud_copy);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    crop_filter.filter(*cropped_cloud);

    RCLCPP_INFO(get_logger(), "Cropped to %zu points. Clustering...", cropped_cloud->size());

    //use euclidien clustering to find legs in cropped_cloud

    ClusterResult leg_clusters;
    clusteringCroppedCloud(cropped_cloud, leg_clusters);

    if (leg_clusters.clusters.size() > 4) {
        RCLCPP_INFO(get_logger(), "More than 4 clusters found (%zu). Correcting...", leg_clusters.clusters.size());
        correctingClusters(leg_clusters, cropped_cloud);
    }

    RCLCPP_INFO(get_logger(), "Found %zu clusters. Building response...", leg_clusters.centroids.size());

    res->success = true;
    res->message = "Found " + std::to_string(leg_clusters.centroids.size()) + " leg clusters";
    res->leg_centroids.clear();
    res->leg_ids.clear();
    for (size_t i = 0; i < leg_clusters.centroids.size(); ++i) {
        geometry_msgs::msg::Point32 pt;
        pt.x = leg_clusters.centroids[i].x();
        pt.y = leg_clusters.centroids[i].y();
        pt.z = leg_clusters.centroids[i].z();
        res->leg_centroids.push_back(pt);
        res->leg_ids.push_back(static_cast<int32_t>(i));
    }
    publishCentroidsMarkers(leg_clusters.centroids);

    RCLCPP_INFO(get_logger(), "Service completed successfully");
}


void LegLocalizer::publishCentroidsMarkers(const std::vector<Eigen::Vector4f>& centroids)
{
  visualization_msgs::msg::MarkerArray arr;

 
  for (size_t i = 0; i < centroids.size(); ++i)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = this->now();
    m.ns = "leg_centroids";
    m.id = static_cast<int>(i);   // stable unique per container
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;

    m.pose.position.x = centroids[i].x();
    m.pose.position.y = centroids[i].y();
    m.pose.position.z = centroids[i].z();
    m.pose.orientation.w = 1.0;

    // sphere diameter
    m.scale.x = 0.08;
    m.scale.y = 0.08;
    m.scale.z = 0.08;

    // color (yellow)
    m.color.r = 1.0f;
    m.color.g = 1.0f;
    m.color.b = 0.0f;
    m.color.a = 0.9f;

    // keep until overwritten
    m.lifetime = rclcpp::Duration::from_seconds(0.0);

    arr.markers.push_back(m);
  }

  pub_->publish(arr);
}
