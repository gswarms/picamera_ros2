// recorder.cpp
#include "../include/picamera_ros2/recorder.hpp"
#include "rclcpp_components/register_node_macro.hpp"

Recorder::Recorder(const rclcpp::NodeOptions & options)
: Node("recorder", options)
{
  this->declare_parameter<std::string>("image_topic_name", "/camera/image_raw");
  this->get_parameter("image_topic_name", image_topic_name_);

  this->declare_parameter<std::string>("px4_mode_topic_name", "/fmu/out/vehicle_status");
  this->get_parameter("px4_mode_topic_name", px4_mode_topic_name_);

  this->declare_parameter<std::string>("interception_mode_topic_name", "/interception/state");
  this->get_parameter("interception_mode_topic_name", interception_mode_topic_name_);

  this->declare_parameter<uint8_t>("px4_record_state", 14);
  this->get_parameter("px4_record_state", px4_record_state_);

  this->declare_parameter<std::string>("interception_record_state", "INTERCEPT");
  this->get_parameter("interception_record_state", interception_record_state_);

  initialized_ = false;
  px4_mode_ = 0;
  interception_mode_ = "IDLE";

  // Subscription to the image topic
  image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
    image_topic_name_, 1, std::bind(&Recorder::image_callback, this, std::placeholders::_1));

  // Subscription to the vehicle status topic
  rclcpp::QoS qos_profile_(rclcpp::KeepLast(1));
  qos_profile_.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  qos_profile_.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  vehicle_status_subscription_ = create_subscription<px4_msgs::msg::VehicleStatus>(
    px4_mode_topic_name_, qos_profile_, std::bind(&Recorder::vehicle_status_callback, this, std::placeholders::_1));

  // Subscription to the interception mode topic
  interception_mode_subscription_ = create_subscription<std_msgs::msg::String>(
    interception_mode_topic_name_, 1, std::bind(&Recorder::interception_mode_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Recorder node initialized");
  // print configuration + parameters
  RCLCPP_INFO(this->get_logger(), "image_topic_name: %s", image_topic_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "px4_mode_topic_name: %s", px4_mode_topic_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "interception_mode_topic_name: %s", interception_mode_topic_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "px4_record_state_: %d", px4_record_state_);
  RCLCPP_INFO(this->get_logger(), "interception_record_state_: %s", interception_record_state_.c_str());
  
}

Recorder::~Recorder()
{
  if (initialized_)
  {
    this->stop_recording();
  }
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

void Recorder::vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
  if (px4_mode_ != msg->nav_state)
  {
    RCLCPP_INFO(this->get_logger(), "nav_mode changed to: %d", msg->nav_state);
    px4_mode_ = msg->nav_state;
  }
}

void Recorder::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  // print conditions for recording
  RCLCPP_DEBUG(this->get_logger(), "px4_mode_: %d", px4_mode_);
  RCLCPP_DEBUG(this->get_logger(), "px4_record_state_: %d", px4_record_state_);
  RCLCPP_DEBUG(this->get_logger(), "interception_mode_: %s", interception_mode_.c_str());
  RCLCPP_DEBUG(this->get_logger(), "interception_record_state_: %s", interception_record_state_.c_str());
  // print initialize
  RCLCPP_DEBUG(this->get_logger(), "initialized_: %d", initialized_);

  if ((px4_mode_ == px4_record_state_) && (interception_mode_ == interception_record_state_))
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

    writer_->write(serialized_msg, image_topic_name_, "sensor_msgs/msg/Image", time_stamp);
  } else {
    if (initialized_)
    {
      this->stop_recording();
    }
  }
}

void Recorder::interception_mode_callback(const std_msgs::msg::String::SharedPtr msg)
{
  interception_mode_ = msg->data;
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
