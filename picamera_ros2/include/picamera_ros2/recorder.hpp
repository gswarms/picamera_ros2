// recorder.hpp
#ifndef RECORDER_HPP
#define RECORDER_HPP

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

class Recorder : public rclcpp::Node
{
public:
  explicit Recorder(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~Recorder();

private:
  void vehicle_status_callback(const std_msgs::msg::String::SharedPtr msg);
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void initialize_recorder();
  void stop_recording();

  std::string topic_name_;
  std::string state_topic_name_;
  std::string bag_name_;
  std::string nav_mode_;
  bool initialized_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr vehicle_status_subscription_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
};

#endif  // RECORDER_HPP
