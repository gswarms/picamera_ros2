#ifndef _PICAMERA_ROS2_HPP_
#define _PICAMERA_ROS2_HPP_

#include "lccv.hpp"
#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
// #include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <picamera_param/picamera_param.hpp>

namespace picamera_ros
{
using namespace lccv;

class PiCameraROS: public rclcpp::Node
{
public:
    PiCameraROS(const rclcpp::NodeOptions &options_);
    ~PiCameraROS();

    void timerCallback();

private:
    PiCamera *camera_;

    std::shared_ptr<picamera_parameters::ParamListener> param_listener_;
    picamera_parameters::Params params_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
};

}

#endif