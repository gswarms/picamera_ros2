// recorder.cpp
#include "../include/picamera_ros2/recorder.hpp"
#include "rclcpp_components/register_node_macro.hpp"

Recorder::Recorder(const rclcpp::NodeOptions & options)
: Node("recorder", options), armed_(false)
{
  // Declare parameters for topic name and bag name
  topic_name_ = this->declare_parameter<std::string>("topic_name", "/camera/image_raw");
  bag_name_ = this->declare_parameter<std::string>("bag_name", "my_bag");

  writer_ = std::make_unique<rosbag2_cpp::Writer>();
  writer_->open(bag_name_);

  // Subscription to the image topic
  image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
    topic_name_, 10, std::bind(&Recorder::image_callback, this, std::placeholders::_1));

  // Subscription to vehicle state topic (e.g., armed state)
  vehicle_status_subscription_ = create_subscription<px4_msgs::msg::VehicleStatus>(
    "/fmu/out/vehicle_state", 10, std::bind(&Recorder::vehicle_status_callback, this, std::placeholders::_1));
}

Recorder::~Recorder()
{
  writer_->~Writer();
}

void Recorder::vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
  armed_ = (msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
}

void Recorder::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
{
  if (armed_)
  {
    rclcpp::Time time_stamp = this->now();

    // Serialize the image message and write to the bag
    rclcpp::SerializedMessage serialized_msg;
    rclcpp::Serialization<sensor_msgs::msg::Image> serializer;
    serializer.serialize_message(msg.get(), &serialized_msg);

    writer_->write(serialized_msg, topic_name_, "sensor_msgs/msg/Image", time_stamp);
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
