// Read-only, finite domain-186 trace. MessageInfo is needed because the
// installed Humble rclpy executor discards publisher GIDs before callbacks.
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

static std::string gid(const uint8_t *data) {
  std::ostringstream out;
  for (size_t i = 0; i < RMW_GID_STORAGE_SIZE; ++i)
    out << std::hex << std::setfill('0') << std::setw(2) << unsigned(data[i]);
  return out.str();
}
static int64_t stamp(const builtin_interfaces::msg::Time &value) {
  return int64_t(value.sec) * 1000000000LL + value.nanosec;
}
static int64_t wall() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
static std::string escaped(const std::string &value) {
  std::string out;
  for (const auto c : value) {
    if (c == '"' || c == '\\') out += '\\';
    if (static_cast<unsigned char>(c) >= 32) out += c;
  }
  return out;
}
int main(int argc, char **argv) {
  const auto domain = std::getenv("ROS_DOMAIN_ID");
  const auto local = std::getenv("ROS_LOCALHOST_ONLY");
  if (!domain || std::string(domain) != "186" || !local || std::string(local) != "1") return 2;
  const int duration = argc > 1 ? std::stoi(argv[1]) : 90;
  if (duration < 5 || duration > 180) return 2;
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("gazebo_tf_ingress_probe",
    rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)}));
  auto metadata = [&](const char *kind, const rclcpp::MessageInfo &info) {
    const auto &rmw = info.get_rmw_message_info();
    std::ostringstream out;
    out << "{\"kind\":\"" << kind << "\",\"wall_monotonic_ns\":" << wall()
        << ",\"receipt_ros_ns\":" << node->get_clock()->now().nanoseconds()
        << ",\"publisher_gid\":\"" << gid(rmw.publisher_gid.data)
        << "\",\"dds_source_timestamp_ns\":" << rmw.source_timestamp
        << ",\"dds_received_timestamp_ns\":" << rmw.received_timestamp;
    return out.str();
  };
  auto tf = node->create_subscription<tf2_msgs::msg::TFMessage>("/tf", rclcpp::SensorDataQoS(),
    [&](const tf2_msgs::msg::TFMessage &message, const rclcpp::MessageInfo &info) {
      for (const auto &t : message.transforms) {
        if (!((t.header.frame_id == "map" && t.child_frame_id == "odom") ||
              (t.header.frame_id == "odom" && t.child_frame_id == "base_footprint"))) continue;
        std::cout << metadata("tf", info) << ",\"parent\":\"" << escaped(t.header.frame_id)
                  << "\",\"child\":\"" << escaped(t.child_frame_id)
                  << "\",\"stamp_ns\":" << stamp(t.header.stamp) << "}" << std::endl;
      }
    });
  auto scan = node->create_subscription<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS(),
    [&](const sensor_msgs::msg::LaserScan &message, const rclcpp::MessageInfo &info) {
      std::cout << metadata("scan", info) << ",\"stamp_ns\":" << stamp(message.header.stamp)
                << ",\"frame\":\"" << escaped(message.header.frame_id) << "\"}" << std::endl;
    });
  const auto began = std::chrono::steady_clock::now();
  while (rclcpp::ok() && std::chrono::steady_clock::now() - began < std::chrono::seconds(duration)) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  rclcpp::shutdown();
}
