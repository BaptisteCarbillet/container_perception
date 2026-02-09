#include <container_perception/leg_detector.hpp>


LegDetector::LegDetector()
: Node("leg_detector"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
{
    n_points_threshold_ = this->declare_parameter<int>("n_points_threshold", 1000);
    global_frame_ = this->declare_parameter<std::string>("global_frame", "map");

    publish_in_rvizz_ = this->declare_parameter<bool>("publish_in_rviz", false);

    ///Normally you would use the frame in the header of the published msg of the d435 camera, which is dpeth_optical_frame, but it doesnt follow the REP 103 so
    /// for simplicty we use the camera_depth_frame which follows REP 103
    sensor_frame_ = this->declare_parameter<std::string>("sensor_frame", "camera_depth_frame");

    delta_crop_ = this->declare_parameter<float>("delta_crop",0.0);

    auto cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_options = rclcpp::SubscriptionOptions();
    sub_options.callback_group = cb_group;


    sub_d435_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/camera/depth/color/points", rclcpp::SensorDataQoS(),
        std::bind(&LegDetector::callback_d435, this, std::placeholders::_1),sub_options);

    sub_containers_ = this->create_subscription<container_perception::msg::PolygonArray>(
        "/container_tracker/containers", rclcpp::SensorDataQoS(),
        std::bind(&LegDetector::callback_containers, this, std::placeholders::_1),sub_options);


    pub_detected_leg_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "detected_leg",
        rclcpp::SensorDataQoS()
    );

    cropbox_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "leg_detector/cropbox",
        rclcpp::SensorDataQoS());


    //std::vector<Leg> tracked_legs_;


}

void LegDetector::callback_containers(const container_perception::msg::PolygonArray& msg) {
    RCLCPP_INFO(this->get_logger(), "containers cb enter");
    {
    std::lock_guard<std::mutex> lk(containers_mtx_);
    tracked_containers_ = msg;
    }
    RCLCPP_INFO(this->get_logger(), "containers cb exit");
}



void LegDetector::setCropBox(const geometry_msgs::msg::Polygon& poly, Eigen::Vector4f& crop_min, Eigen::Vector4f& crop_max) {
    float min_x = std::numeric_limits<float>::max();
    //float min_y = std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    //float max_y = std::numeric_limits<float>::lowest();
    float max_z = std::numeric_limits<float>::lowest();
    for (const auto & p : poly.points) {
        if (p.x < min_x) min_x = p.x;
        if (p.x > max_x) max_x = p.x;

        if (p.z < min_z) min_z = p.z;
        if (p.z > max_z) max_z = p.z;
        //if (p.x < min_x) min_x = p.x;
        //if (p.y < min_y) min_y = p.y;
        

        //if (p.x > max_x) max_x = p.x;
        //if (p.y > max_y) max_y = p.y;
        

    }
    //min_z = -100.0f;
    //max_z = 100.0f;
    //crop_min = Eigen::Vector4f(min_x - delta_crop_, min_y - delta_crop_, min_z, 1.0f);
    //crop_max = Eigen::Vector4f(max_x + delta_crop_, max_y + delta_crop_, max_z, 1.0f);

    //the sensor frames aka depth_optical_frame doesnt follow REP 103, which makes the cropping weird
    // In camera_depth_optical_frame: x=right, y=down, z=forward.
    //RCLCPP_INFO(this->get_logger(),"%.6f",poly.points[0].y);
    float min_y = poly.points[0].y;//-10.0f;
    float max_y =  0.28f;
     crop_min = Eigen::Vector4f(min_x - delta_crop_,
                               min_y,
                               min_z - delta_crop_,
                               1.0f);

    crop_max = Eigen::Vector4f(max_x + delta_crop_,
                               max_y,
                               max_z + delta_crop_,
                               1.0f);

}


void LegDetector::callback_d435(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

    RCLCPP_INFO(this->get_logger(),"d435 callback received");
    if(msg->height * msg->width < static_cast<std::size_t>(n_points_threshold_)) {
        RCLCPP_WARN(this->get_logger(),"No d435 callback, msg too small");
        return;
    }
    
    const auto &target = msg->header.frame_id;   // cloud frame
    const auto &source = global_frame_;          // e.g. "map"
    const auto stamp = msg->header.stamp;


    if (!tf_buffer_.canTransform(target, source, stamp, tf2::durationFromSec(0.0))) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "TF not available yet %s <- %s at stamp, skipping frame",
                        target.c_str(), source.c_str());
    return;
    }

    try {
        tf_2_sensor_ = tf_buffer_.lookupTransform(
        target,
        source,
        stamp
        //tf2::durationFromSec(0.05)
        );
        //tf2::durationFromSec(0.3));
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s",
                    msg->header.frame_id.c_str(), global_frame_.c_str(), ex.what());
        return;
    }

    container_perception::msg::PolygonArray containers_in_sensor_frame;
    container_perception::msg::PolygonArray containers_copy;
    RCLCPP_INFO(this->get_logger(), "cloud cb before lock");
    {
       std::lock_guard<std::mutex> lk(containers_mtx_);
       containers_copy = tracked_containers_;
    }
    RCLCPP_INFO(this->get_logger(), "cloud cb after lock");
    
    for (const auto& container : containers_copy.polygons) {
    geometry_msgs::msg::Polygon polygon_in_sensor_frame;
    tf2::doTransform(container,polygon_in_sensor_frame,tf_2_sensor_);
    containers_in_sensor_frame.polygons.push_back(polygon_in_sensor_frame);

    }
    for (const auto& id : containers_copy.ids) {
        containers_in_sensor_frame.ids.push_back(id);
    } 


    // Convert to PCL
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr in(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *in);

    if (in->empty()) {
      return;
    }

    //Cropping cloud by containers // TO DO : find a smarter way
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::CropBox<pcl::PointXYZRGB> crop;
    Eigen::Vector4f crop_min;
    Eigen::Vector4f crop_max;
    
    
    for (size_t i = 0; i < containers_in_sensor_frame.polygons.size(); ++i) {
    
        const auto& container = containers_in_sensor_frame.polygons[i];
        int container_id = containers_in_sensor_frame.ids[i];
        setCropBox(container, crop_min, crop_max);
        crop.setMin(crop_min);
        crop.setMax(crop_max);
        crop.setInputCloud(in);
        //crop.setTransf

        crop.filter(*out);
        RCLCPP_INFO(this->get_logger(),"Container ID: %d", container_id);
        
        
        if (out->size() > static_cast<std::size_t>(n_points_threshold_)) {

            if (publish_in_rvizz_) {
            publishCropBoxMarker(crop_min, crop_max, msg->header.frame_id, container_id); 
            }
            RCLCPP_INFO(this->get_logger(),"cropped cloud size : %d", static_cast<int>(out->size()));
            
            //find centroid of out 
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*out,centroid);

            int leg_id = -1;
            findLegId(container, centroid, leg_id);

            Leg leg;
            leg.container_id = container_id;
            leg.leg_id = leg_id;
            float angle = -1.0f;
            computeLegAngle(out, angle, leg_id, container_id);
            leg.angle = angle;
            tracked_legs_.push_back(leg);

            if (publish_in_rvizz_) {
                sensor_msgs::msg::PointCloud2 out_msg;
                pcl::toROSMsg(*out, out_msg);
                out_msg.header.frame_id = msg->header.frame_id;
                out_msg.header.stamp = this->now();
                publishLeg(out_msg);
            }

            return;

        }


    }

}
    

void LegDetector::findLegId(const geometry_msgs::msg::Polygon& poly, Eigen::Vector4f centroid, int& leg_id) {
    //Find the leg_id, ie find the polygon points index that is closest to the centroid
    
    float min_dist = std::numeric_limits<float>::max();;
    for (size_t i = 0; i < poly.points.size(); ++i) {
        const auto& p = poly.points[i];
        float dist = std::sqrt(std::pow(p.x - centroid.x(),2) +
                               std::pow(p.y - centroid.y(),2) +
                               std::pow(p.z - centroid.z(),2));
        if (dist < min_dist) {
            min_dist = dist;
            leg_id = static_cast<int>(i);   
        }
    }
}


void LegDetector::publishLeg(const sensor_msgs::msg::PointCloud2& msg) {
    pub_detected_leg_->publish(msg);
}



    
    
    
    
void LegDetector::publishCropBoxMarker(const Eigen::Vector4f& crop_min,
                                       const Eigen::Vector4f& crop_max,
                                       const std::string& frame_id,
                                       int id)
{
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = frame_id;
    marker.header.stamp = this->now();
    marker.ns = "cropbox";
    marker.id = id;                       // unique id per container
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Center of the box
    marker.pose.position.x = 0.5f * (crop_min.x() + crop_max.x());
    marker.pose.position.y = 0.5f * (crop_min.y() + crop_max.y());
    marker.pose.position.z = 0.5f * (crop_min.z() + crop_max.z());

    // No rotation
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Scale is box size along each axis
    marker.scale.x = (crop_max.x() - crop_min.x());
    marker.scale.y = (crop_max.y() - crop_min.y());
    marker.scale.z = (crop_max.z() - crop_min.z());

    // Color: semi-transparent green
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.3f;   // alpha MUST > 0

    marker.lifetime = rclcpp::Duration::from_seconds(0.0); // keep until overwritten

    cropbox_marker_pub_->publish(marker);
}


void LegDetector::computeLegAngle(pcl::PointCloud<pcl::PointXYZRGB>::Ptr leg_cloud, float& angle, int leg_id, int container_id) {
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);

    ne.setSearchMethod (tree);
    ne.setInputCloud (leg_cloud);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (0.05);
    seg.setRadiusLimits (0.0, 0.15);
    seg.setInputCloud (leg_cloud);
    seg.setInputNormals (cloud_normals);
    seg.segment (*inliers_cylinder, *coefficients_cylinder);

    if (inliers_cylinder->indices.size () == 0)
    {
        RCLCPP_WARN(this->get_logger(),"Could not estimate a cylinder for the leg");
        angle = -1.0f;
        return;
    }
    Eigen::Vector3f axis_vector;
    axis_vector[0] = coefficients_cylinder->values[3];
    axis_vector[1] = coefficients_cylinder->values[4];
    axis_vector[2] = coefficients_cylinder->values[5];
    axis_vector.normalize();

    Eigen::Vector3f vertical_vector(0.0f, 0.0f, 1.0f);
    angle = std::acos(axis_vector.dot(vertical_vector)) * 180.0f / M_PI;
    RCLCPP_INFO(this->get_logger(),"Leg angle for leg_id %d of container_id %d : %.2f degrees", leg_id, container_id, angle);
}