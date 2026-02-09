#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/path.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <container_perception/srv/localize_legs.hpp>

#include <cmath>
#include <limits>
#include <vector>
#include <string>
#include <optional>

#include <rs_cloud/srv/save_cloud.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/msg/state.hpp>



class LegInspectorAsync : public rclcpp_lifecycle::LifecycleNode
{
public:
  using ComputePath    = nav2_msgs::action::ComputePathToPose;
  using NavToPose      = nav2_msgs::action::NavigateToPose;
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


  explicit LegInspectorAsync(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : LifecycleNode("leg_inspector", options)
  {
    // ---- Params ----
    map_frame_  = declare_parameter<std::string>("map_frame", "map");
    container_id_ = declare_parameter<int32_t>("container_id", 0);

    views_per_leg_    = declare_parameter<int>("views_per_leg", 3);
    min_angle_sep_deg_ = declare_parameter<double>("min_angle_sep_deg", 60.0);

    r_min_      = declare_parameter<double>("r_min", 0.9);
    r_max_      = declare_parameter<double>("r_max", 2.0);
    dr_         = declare_parameter<double>("dr", 0.2);
    dtheta_deg_ = declare_parameter<double>("dtheta_deg", 15.0);

    plan_timeout_sec_ = declare_parameter<double>("plan_timeout_sec", 0.8);
    nav_timeout_sec_  = declare_parameter<double>("nav_timeout_sec", 120.0);

    compute_path_action_ = declare_parameter<std::string>("compute_path_action", "/compute_path_to_pose");
    navigate_action_     = declare_parameter<std::string>("navigate_action", "/navigate_to_pose");
    leg_service_name_    = declare_parameter<std::string>("leg_service", "/localize_legs");
    planner_id_          = declare_parameter<std::string>("planner_id", ""); // empty => default

    hold_after_goal_sec_ = declare_parameter<double>("hold_after_goal_sec", 2.0);

    // ---- Callback groups ----
    //cbg_clients_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    //cbg_main_    = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    //cbg_          = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // ---- Service client in Mutually exclusive group ----
    //leg_client_ = this->create_client<container_perception::srv::LocalizeLegs>(
    //  leg_service_name_,
    //  rmw_qos_profile_services_default,   // QoS profile
    //  cbg_                                // callback group
    //);

    // ---- Action clients in Mutually exclusive group ----
    //compute_path_client_ = rclcpp_action::create_client<ComputePath>(
    //  this,
    //  compute_path_action_,
    //  cbg_                                // callback group
    //);

    //navigate_client_ = rclcpp_action::create_client<NavToPose>(
    //  this,
    //  navigate_action_,
    //  cbg_                                // callback group
    //);

    // ---- Timer in main group ----
   //  start_timer_ = this->create_wall_timer(
    //  std::chrono::milliseconds(250),
    //  std::bind(&LegInspectorAsync::tick_start_, this),
    //  cbg_
    //);

    save_service_name_ = declare_parameter<std::string>("save_service", "/save_cloud");
    save_delay_sec_    = declare_parameter<double>("save_delay_sec", 1.0);

    //save_client_ = this->create_client<rs_cloud::srv::SaveCloud>(
    //  save_service_name_,
    //  rmw_qos_profile_services_default,
    //  cbg_
    //);  
    RCLCPP_INFO(get_logger(), "LegInspectorAsync (first-reachable) ready.");
    //Print parameters r_min, r_max, views_per_leg
    RCLCPP_INFO(get_logger(), "r_min=%.2f, r_max=%.2f, views_per_leg=%d", r_min_, r_max_, views_per_leg_);
}

private:

  // --------------------------
  // Lifecycle transitions
  // --------------------------
  CallbackReturn on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "LegInspectorAsync Configuring...");
    cbg_          = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // ---- Service client in Mutually exclusive group ----
    leg_client_ = this->create_client<container_perception::srv::LocalizeLegs>(
      leg_service_name_,
      rmw_qos_profile_services_default,   // QoS profile
      cbg_                                // callback group
    );

    // ---- Action clients in Mutually exclusive group ----
    compute_path_client_ = rclcpp_action::create_client<ComputePath>(
      this,
      compute_path_action_,
      cbg_                                // callback group
    );

    navigate_client_ = rclcpp_action::create_client<NavToPose>(
      this,
      navigate_action_,
      cbg_                                // callback group
    );

    // ---- Timer in main group ----
   // start_timer_ = this->create_wall_timer(
   //   std::chrono::milliseconds(250),
    //  std::bind(&LegInspectorAsync::tick_start_, this),
    //  cbg_
    //);
    done_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "leg_inspector/done", rclcpp::QoS(10)
    );
    

    save_client_ = this->create_client<rs_cloud::srv::SaveCloud>(
      save_service_name_,
      rmw_qos_profile_services_default,
      cbg_
    ); 
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "LegInspectorAsync Activating...");
    // Start timer
    container_id_ = this->get_parameter("container_id").as_int();
    RCLCPP_INFO(get_logger(), "Using container_id=%d", container_id_);
    started_ = false;
    start_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(250),
      std::bind(&LegInspectorAsync::tick_start_, this),
      cbg_
    );

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "LegInspectorAsync Deactivating...");
    // Stop timer
    if (start_timer_) {
      start_timer_->cancel();
      start_timer_.reset();
    }
    cancel_nav_timeout_();
    cancel_plan_timeout_();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "LegInspectorAsync Cleaning up...");
    // Clear state
    leg_ids_.clear();
    leg_pts_.clear();
    leg_idx_ = 0;
    candidates_.clear();
    cand_idx_ = 0;
    planning_in_flight_ = false;
    plan_goal_handle_.reset();
    navigating_in_flight_ = false;
    nav_goal_handle_.reset();
    accepted_angles_rad_.clear();
    reachable_views_.clear();
    view_idx_ = 0;

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "LegInspectorAsync Shutting down...");
    on_deactivate(rclcpp_lifecycle::State());
    return CallbackReturn::SUCCESS;
  }


  // -------------------------
  // Startup / interface wait
  // -------------------------
  void tick_start_()
  {
    if (started_) return;

    const bool srv_ok  = leg_client_->service_is_ready();
    const bool plan_ok = compute_path_client_->action_server_is_ready();
    const bool nav_ok  = navigate_client_->action_server_is_ready();
    const bool save_ok = save_client_->service_is_ready();

    if (!srv_ok || !plan_ok || !nav_ok || !save_ok) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "Waiting... service=%d compute_path=%d navigate=%d save=%d",
        srv_ok, plan_ok, nav_ok, save_ok);
      return;
    }

    started_ = true;
    start_timer_->cancel();

    RCLCPP_INFO(get_logger(), "Interfaces ready. Requesting legs for container %d", container_id_);
    request_legs_();
  }

  // -------------------------
  // Step 1: service call
  // -------------------------
  void request_legs_()
  {
    auto req = std::make_shared<container_perception::srv::LocalizeLegs::Request>();
    req->container_id = container_id_;

    leg_client_->async_send_request(req,
      [this](rclcpp::Client<container_perception::srv::LocalizeLegs>::SharedFuture fut)
      {
        auto resp = fut.get();

        if (!resp->success) {
          RCLCPP_WARN(get_logger(), "Leg service error: %s", resp->message.c_str());
          finish_("Leg localization failed");
          return;
        }

        if (resp->leg_ids.size() != resp->leg_centroids.size()) {
          RCLCPP_ERROR(get_logger(), "Mismatched response sizes: leg_ids=%zu centroids=%zu",
                       resp->leg_ids.size(), resp->leg_centroids.size());
          finish_("Bad leg response");
          return;
        }

        leg_ids_ = resp->leg_ids;
        leg_pts_ = resp->leg_centroids;
        leg_idx_ = 0;

        RCLCPP_INFO(get_logger(), "Got %zu legs", leg_ids_.size());
        inspect_next_leg_();
      });
  }

  // -------------------------
  // Step 2: per-leg state
  // -------------------------
  void inspect_next_leg_()

  {
    
    accepted_angles_rad_.clear();
    reachable_views_.clear();
    view_idx_ = 0;


    cancel_plan_timeout_();
    cancel_nav_timeout_();

    if (leg_idx_ >= leg_ids_.size()) {
      finish_("All legs inspected.");
      std_msgs::msg::Bool done_msg;
      done_msg.data = true;
      done_pub_->publish(done_msg);

      return;
    }

    const int32_t leg_id = leg_ids_[leg_idx_];
    const auto & p = leg_pts_[leg_idx_];

    RCLCPP_INFO(get_logger(), "Inspect leg %d (%zu/%zu) at (%.2f, %.2f)",
                leg_id, leg_idx_ + 1, leg_ids_.size(), p.x, p.y);

    build_candidates_(p.x, p.y);
    cand_idx_ = 0;

    // Find first reachable candidate (async loop)
    eval_next_candidate_();
  }

  void build_candidates_(double leg_x, double leg_y)
  {
    candidates_.clear();
    const double dtheta = dtheta_deg_ * M_PI / 180.0;

    for (double r = r_min_; r <= r_max_ + 1e-9; r += dr_) {
      for (double th = 0.0; th < 2.0 * M_PI - 1e-9; th += dtheta) {
        const double vx = leg_x + r * std::cos(th);
        const double vy = leg_y + r * std::sin(th);
        candidates_.push_back(make_pose_facing_(vx, vy, leg_x, leg_y));
      }
    }

    RCLCPP_INFO(get_logger(), "Generated %zu candidates", candidates_.size());
  }

  geometry_msgs::msg::PoseStamped make_pose_facing_(double vx, double vy, double tx, double ty)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = map_frame_;
    pose.header.stamp = now();
    pose.pose.position.x = vx;
    pose.pose.position.y = vy;
    pose.pose.position.z = 0.0;

    const double yaw = std::atan2(ty - vy, tx - vx);
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    pose.pose.orientation = tf2::toMsg(q);
    return pose;
  }

  // -------------------------
  // Step 3: Evaluate candidates (ComputePathToPose)
  // pick FIRST candidate with non-empty path
  // -------------------------
  void eval_next_candidate_()
  {
    if (cand_idx_ >= candidates_.size()) {
      RCLCPP_WARN(get_logger(), "No more candidates (%zu/%zu). Got %zu views so far.",
                  cand_idx_, candidates_.size(), reachable_views_.size());
      leg_idx_++;
      inspect_next_leg_();
      return;
    }

    const auto pose = candidates_[cand_idx_++];
    send_compute_path_goal_(pose);
  }

  void send_compute_path_goal_(const geometry_msgs::msg::PoseStamped & pose)
  {
    if (planning_in_flight_) {
      eval_next_candidate_();
      return;
    }

    if (!compute_path_client_->action_server_is_ready()) {
      RCLCPP_WARN(get_logger(), "ComputePath action server not ready, skipping candidate");
      eval_next_candidate_();
      return;
    }

    planning_in_flight_ = true;
    plan_goal_handle_.reset();

    ComputePath::Goal goal;
    goal.goal = pose;
    goal.planner_id = planner_id_;
    goal.use_start = false;

    rclcpp_action::Client<ComputePath>::SendGoalOptions opts;

    opts.goal_response_callback =
      [this](rclcpp_action::ClientGoalHandle<ComputePath>::SharedPtr gh)
      {
        if (!gh) {
          planning_in_flight_ = false;
          cancel_plan_timeout_();
          eval_next_candidate_();
          return;
        }
        plan_goal_handle_ = gh;
        start_plan_timeout_();
      };

    opts.result_callback =
      [this, pose](const rclcpp_action::ClientGoalHandle<ComputePath>::WrappedResult & res)
      {
        RCLCPP_DEBUG(get_logger(), "Planning result received: code=%d", (int)res.code);
        if (!planning_in_flight_) return;  // late result after timeout

        planning_in_flight_ = false;
        cancel_plan_timeout_();

        bool reachable = false;
        nav_msgs::msg::Path path;

        if (res.code == rclcpp_action::ResultCode::SUCCEEDED && res.result) {
          path = res.result->path;
          reachable = !path.poses.empty();
        }

        if (reachable) {

          if (accept_view_(pose)) {
            RCLCPP_INFO(get_logger(), "Found reachable view for leg %d",
                         leg_ids_[leg_idx_]);
            reachable_views_.push_back(pose);
            
          } else {
            RCLCPP_INFO(get_logger(), "Reachable, but too close in angle, skipping view for leg %d",
                         leg_ids_[leg_idx_]);
          }

          if (static_cast<int>(reachable_views_.size()) >= views_per_leg_) {
            RCLCPP_INFO(get_logger(), "Collected %d views for leg %d, starting navigation",
                        views_per_leg_, leg_ids_[leg_idx_]);
            view_idx_ = 0;
            navigate_to_pose_(reachable_views_[view_idx_]);
            return;
          }


        }
        
        // try next candidate
        eval_next_candidate_();
        
      };

    compute_path_client_->async_send_goal(goal, opts);
  }

  void start_plan_timeout_()
  {
    cancel_plan_timeout_();
    plan_timeout_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(plan_timeout_sec_)),
      std::bind(&LegInspectorAsync::on_plan_timeout_, this),
      cbg_);
  }

  void on_plan_timeout_()
  {
    cancel_plan_timeout_();
    if (!planning_in_flight_) return;

    planning_in_flight_ = false;
    if (plan_goal_handle_) {
      compute_path_client_->async_cancel_goal(plan_goal_handle_);
    }
    // just move on
    eval_next_candidate_();
  }

  void cancel_plan_timeout_()
  {
    if (plan_timeout_timer_) {
      plan_timeout_timer_->cancel();
      plan_timeout_timer_.reset();
    }
  }

  // -------------------------
  // Step 4: NavigateToPose
  // -------------------------
  void navigate_to_pose_(const geometry_msgs::msg::PoseStamped & goal_pose)
  {
    if (navigating_in_flight_) {
      // If this ever triggers, you can choose to cancel previous navigation first
      RCLCPP_WARN(get_logger(), "Navigation already in flight, skipping.");
      return;
    }

    navigating_in_flight_ = true;
    nav_goal_handle_.reset();

    NavToPose::Goal goal;
    goal.pose = goal_pose;

    rclcpp_action::Client<NavToPose>::SendGoalOptions opts;

    opts.goal_response_callback =
      [this](rclcpp_action::ClientGoalHandle<NavToPose>::SharedPtr gh)
      {
        if (!gh) {
          navigating_in_flight_ = false;
          cancel_nav_timeout_();
          RCLCPP_WARN(get_logger(), "Navigate goal rejected for leg %d", leg_ids_[leg_idx_]);
          leg_idx_++;
          inspect_next_leg_();
          return;
        }

        nav_goal_handle_ = gh;
        start_nav_timeout_();
      };

    opts.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<NavToPose>::WrappedResult & res)
      {
        if (!navigating_in_flight_) return;

        navigating_in_flight_ = false;
        cancel_nav_timeout_();

        if (res.code == rclcpp_action::ResultCode::SUCCEEDED) {
          const int32_t leg_id = leg_ids_[leg_idx_];
          const int32_t view_id = view_idx_;
          RCLCPP_INFO(get_logger(), "Arrived at view %d for leg %d",
                      view_id, leg_id);
          schedule_save_after_arrival_(leg_id, view_id);
        } else {
          RCLCPP_WARN(get_logger(), "Navigation failed at view %d for leg %d",
                      view_idx_, leg_ids_[leg_idx_]);
        }
        
        view_idx_++;
        if (view_idx_ < static_cast<int>(reachable_views_.size())) {
          //navigate_to_pose_(reachable_views_[view_idx_]);
          schedule_next_nav_after_hold_();
          
          return;
        }
        
        // Move to next leg
        //leg_idx_++;
        //inspect_next_leg_();
        schedule_next_nav_after_hold_();
      };

    navigate_client_->async_send_goal(goal, opts);
  }

  void start_nav_timeout_()
  {
    cancel_nav_timeout_();
    nav_timeout_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(nav_timeout_sec_)),
      std::bind(&LegInspectorAsync::on_nav_timeout_, this),
      cbg_);
  }

  void on_nav_timeout_()
  {
    cancel_nav_timeout_();
    if (!navigating_in_flight_) return;

    navigating_in_flight_ = false;
    if (nav_goal_handle_) {
      navigate_client_->async_cancel_goal(nav_goal_handle_);
    }

    RCLCPP_WARN(get_logger(), "Navigation timed out for leg %d", leg_ids_[leg_idx_]);

    leg_idx_++;
    inspect_next_leg_();
  }

  void cancel_nav_timeout_()
  {
    if (nav_timeout_timer_) {
      nav_timeout_timer_->cancel();
      nav_timeout_timer_.reset();
    }
  }

  void finish_(const std::string & msg)
  {
    RCLCPP_INFO(get_logger(), "%s", msg.c_str());
    // If you want auto-exit:
    // rclcpp::shutdown();
  }


  bool accept_view_(const geometry_msgs::msg::PoseStamped & pose)
  {
    // Compute view angle around the leg center
    const auto & leg = leg_pts_[leg_idx_];
    const double ang = std::atan2(pose.pose.position.y - leg.y,
                                  pose.pose.position.x - leg.x);

    const double min_sep = min_angle_sep_deg_ * M_PI / 180.0;

    for (double a : accepted_angles_rad_) {
      double d = std::fabs(std::atan2(std::sin(ang - a), std::cos(ang - a))); // wrap to [-pi,pi]
      if (d < min_sep) return false;
    }
    RCLCPP_INFO(get_logger(), "Accepted view angle %.2f deg", ang * 180.0 / M_PI);
    accepted_angles_rad_.push_back(ang);
    return true;
  }

  void schedule_save_after_arrival_(int32_t leg_id, int32_t view_idx)
  {
    // Cancel any previous pending save (optional)
    if (save_timer_) {
      save_timer_->cancel();
      save_timer_.reset();
    }

    // One-shot timer: fire once after delay, then self-cancel
    save_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(save_delay_sec_)),
      [this, leg_id, view_idx]()
      {
        // one-shot behavior
        if (save_timer_) {
          save_timer_->cancel();
          save_timer_.reset();
        }
        request_save_(leg_id, view_idx);
      },
      cbg_
    );
  }

  void request_save_(int32_t leg_id, int32_t view_idx)
  {
    if (!save_client_ || !save_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "SaveCloud service not ready, skipping save.");
      return;
    }

    auto req = std::make_shared<rs_cloud::srv::SaveCloud::Request>();
    req->container_id = container_id_;
    req->leg_id = leg_id;
    req->view_idx = view_idx;

    req->leg_centroid = leg_pts_[leg_idx_];

    save_client_->async_send_request(req,
      [this](rclcpp::Client<rs_cloud::srv::SaveCloud>::SharedFuture fut)
      {
        auto resp = fut.get();
        if (!resp->success) {
          RCLCPP_WARN(get_logger(), "SaveCloud failed: %s", resp->message.c_str());
        } else {
          RCLCPP_INFO(get_logger(), "Saved cloud: %s", resp->filename.c_str());
        }
      });
  }

  void schedule_next_nav_after_hold_()
  {
    if (hold_timer_) {
      hold_timer_->cancel();
      hold_timer_.reset();
    }

    hold_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(hold_after_goal_sec_)),
      [this]()
      {
        if (hold_timer_) {
          hold_timer_->cancel();
          hold_timer_.reset();
        }

        // Continue to next view or next leg
        if (view_idx_ < static_cast<int>(reachable_views_.size())) {
          navigate_to_pose_(reachable_views_[view_idx_]);
        } else {
          leg_idx_++;
          inspect_next_leg_();
        }
      },
      cbg_
    );
  }




private:
  // Params
  std::string map_frame_;
  int32_t container_id_{0};

  double r_min_{0.9}, r_max_{2.0}, dr_{0.2}, dtheta_deg_{15.0};
  double plan_timeout_sec_{0.8}, nav_timeout_sec_{120.0};

  std::string compute_path_action_;
  std::string navigate_action_;
  std::string leg_service_name_;
  std::string planner_id_;

  // Callback groups
  //rclcpp::CallbackGroup::SharedPtr cbg_clients_;
  //rclcpp::CallbackGroup::SharedPtr cbg_main_;
  rclcpp::CallbackGroup::SharedPtr cbg_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr done_pub_;

  // Clients
  rclcpp_action::Client<ComputePath>::SharedPtr compute_path_client_;
  rclcpp_action::Client<NavToPose>::SharedPtr navigate_client_;
  rclcpp::Client<container_perception::srv::LocalizeLegs>::SharedPtr leg_client_;

  // Timers
  rclcpp::TimerBase::SharedPtr start_timer_;
  rclcpp::TimerBase::SharedPtr plan_timeout_timer_;
  rclcpp::TimerBase::SharedPtr nav_timeout_timer_;

  // State
  bool started_{false};

  std::vector<int32_t> leg_ids_;
  std::vector<geometry_msgs::msg::Point32> leg_pts_;
  size_t leg_idx_{0};

  std::vector<geometry_msgs::msg::PoseStamped> candidates_;
  size_t cand_idx_{0};

  // In-flight tracking (planning)
  bool planning_in_flight_{false};
  rclcpp_action::ClientGoalHandle<ComputePath>::SharedPtr plan_goal_handle_;

  // In-flight tracking (navigation)
  bool navigating_in_flight_{false};
  rclcpp_action::ClientGoalHandle<NavToPose>::SharedPtr nav_goal_handle_;

  std::vector<double> accepted_angles_rad_{};

  int views_per_leg_{3};
  double min_angle_sep_deg_{60.0};

  std::vector<geometry_msgs::msg::PoseStamped> reachable_views_{};
  int view_idx_{0};

  // Save service
  std::string save_service_name_;
  double save_delay_sec_{1.0};
  rclcpp::Client<rs_cloud::srv::SaveCloud>::SharedPtr save_client_;
  rclcpp::TimerBase::SharedPtr save_timer_;

  double hold_after_goal_sec_{2.0};
  rclcpp::TimerBase::SharedPtr hold_timer_;


};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LegInspectorAsync>();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
