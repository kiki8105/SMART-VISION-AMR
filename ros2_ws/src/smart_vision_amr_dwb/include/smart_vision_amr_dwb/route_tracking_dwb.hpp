#ifndef SMART_VISION_AMR_DWB__ROUTE_TRACKING_DWB_HPP_
#define SMART_VISION_AMR_DWB__ROUTE_TRACKING_DWB_HPP_

#include <memory>
#include <string>
#include <atomic>
#include <limits>

#include "dwb_core/dwb_local_planner.hpp"
#include "std_msgs/msg/string.hpp"

namespace smart_vision_amr_dwb
{

class RouteTrackingDWB : public dwb_core::DWBLocalPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void activate() override;
  void deactivate() override;
  void cleanup() override;
  dwb_msgs::msg::TrajectoryScore scoreTrajectory(
    const dwb_msgs::msg::Trajectory2D & trajectory, double best_score = -1) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  nav_2d_msgs::msg::Path2D transformGlobalPlan(
    const nav_2d_msgs::msg::Pose2DStamped & pose) override;

  double route_horizon_distance_{0.55};
  double route_anchor_search_distance_{0.85};
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr diagnostics_pub_;
  int64_t last_diagnostic_ns_{0};
  double nominal_linear_limit_{0.22};
  double turn_start_angular_{0.08};
  double turn_full_angular_{0.35};
  double turn_entry_linear_{0.15};
  double turn_min_linear_{0.10};
  double moving_turn_max_angular_{0.35};
  std::atomic<double> linear_speed_limit_{std::numeric_limits<double>::infinity()};
};

}  // namespace smart_vision_amr_dwb
#endif  // SMART_VISION_AMR_DWB__ROUTE_TRACKING_DWB_HPP_
