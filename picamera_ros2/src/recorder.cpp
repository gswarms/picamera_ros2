// recorder.cpp
#include "../include/picamera_ros2/recorder.hpp"
#include "rclcpp_components/register_node_macro.hpp"

Recorder::Recorder(const rclcpp::NodeOptions & options)
: Node("recorder", options)
{
  this->declare_parameter<std::string>("topic_name", "/camera/image_raw");
  this->get_parameter("topic_name", topic_name_);
  this->declare_parameter<std::string>("state_topic_name", "/interceptor/state");
  this->get_parameter("state_topic_name", state_topic_name_);
  this->declare_parameter<std::string>("recording_state", "INTERCEPT");
  this->get_parameter("recording_state", recording_state_);

  initialized_ = false;

  // Subscription to the image topic
  image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
    topic_name_, 1, std::bind(&Recorder::image_callback, this, std::placeholders::_1));

  // Subscription to the vehicle status topic
  vehicle_status_subscription_ = create_subscription<std_msgs::msg::String>(
    state_topic_name_, 1, std::bind(&Recorder::vehicle_status_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Recorder node initialized");
  // print configuration + parameters
  RCLCPP_INFO(this->get_logger(), "topic_name: %s", topic_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "state_topic_name: %s", state_topic_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "recording_state: %s", recording_state_.c_str());
  
}

Recorder::~Recorder()
{
  return;
}

void Recorder::initialize_recorder()
{
  // get current time
  std::time_t t = std::time(0);
  std::tm* now = std::localtime(&t);
  std::string current_time = "/bags/camera_" + std::to_string(now->tm_year + 1900) + "_" + std::to_string(now->tm_mon + 1) + "_" + std::to_string(now->tm_mday) + "-" + std::to_string(now->tm_hour) + "_" + std::to_string(now->tm_min) + "_" + std::to_string(now->tm_sec);

  writer_ = std::make_unique<rosbag2_cpp::Writer>();
  writer_->open(current_time);
  initialized_ = true;
  RCLCPP_INFO(this->get_logger(), "Recording started, bag name: %s", current_time.c_str());
}

void Recorder::stop_recording()
{
  writer_->~Writer();
  initialized_ = false;
  RCLCPP_INFO(this->get_logger(), "Recording stopped");
}

void Recorder::vehicle_status_callback(const std_msgs::msg::String::SharedPtr msg)
{
  nav_mode_ = msg->data;
  auto & clk = *this->get_clock();
  RCLCPP_INFO_THROTTLE(this->get_logger(), clk, 4000, "nav_mode: %s", nav_mode_.c_str());
}

void Recorder::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (strcmp(nav_mode_.c_str(), recording_state_.c_str()) == 0)
  {
    if (!initialized_)
    {
      this->initialize_recorder();
    }
    rclcpp::Time time_stamp = this->now();

    // Serialize the image message and write to the bag
    rclcpp::SerializedMessage serialized_msg;
    rclcpp::Serialization<sensor_msgs::msg::Image> serializer;
    serializer.serialize_message(msg.get(), &serialized_msg);

    writer_->write(serialized_msg, topic_name_, "sensor_msgs/msg/Image", time_stamp);
  } else {
    if (initialized_)
    {
      this->stop_recording();
    }
  }
}

// Register the component for composability
RCLCPP_COMPONENTS_REGISTER_NODE(Recorder)

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions options;
	auto node = std::make_shared<Recorder>(options);

	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
