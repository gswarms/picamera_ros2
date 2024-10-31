// recorder.hpp
#ifndef RECORDER_HPP
#define RECORDER_HPP

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "std_msgs/msg/string.hpp"

class Recorder : public rclcpp::Node
{
public:
  explicit Recorder(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~Recorder();

private:
  void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void interception_mode_callback(const std_msgs::msg::String::SharedPtr msg);
  void initialize_recorder();
  void stop_recording();

  std::string image_topic_name_;
  std::string px4_mode_topic_name_;
  std::string interception_mode_topic_name_;

  std::string bag_name_;

  uint8_t px4_mode_;
  std::string interception_mode_;
  uint8_t px4_record_state_;
  std::string interception_record_state_;

  bool initialized_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr interception_mode_subscription_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
};

#endif  // RECORDER_HPP
