#ifndef _PICAMERA_ROS2_HPP_
#define _PICAMERA_ROS2_HPP_

#include "lccv.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp_components/register_node_macro.hpp>

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
    int video_width_;
    int video_height_;
    int framerate_;
    bool verbose_;
    bool hdr_;
    float shutter;
    bool publish_grayscale_image_;

    rclcpp::Time camera_initilaize_time;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr gray_image_pub_;
};

}

#endif