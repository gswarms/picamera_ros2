#include "picamera_ros2/picamera_pub.hpp"

namespace picamera_ros
{

PiCameraROS::PiCameraROS(const rclcpp::NodeOptions &options_): Node("picamera_ros", options_)
{
    this->camera_ = new lccv::PiCamera();

    this->declare_parameter("video_width", 640);
    this->declare_parameter("video_height", 480);
    this->declare_parameter("framerate", 30);
    this->declare_parameter("shutter", 1000.0);
    this->declare_parameter("hdr", false);
    this->declare_parameter("verbose", false);

    this->get_parameter("video_width", this->video_width_);
    this->get_parameter("video_height", this->video_height_);
    this->get_parameter("framerate", this->framerate_);
    this->get_parameter("hdr", this->hdr_);
    this->get_parameter("shutter", this->shutter);
    this->get_parameter("verbose", this->verbose_);

    this->camera_->options->video_width = this->video_width_;
    this->camera_->options->video_height = this->video_height_;
    this->camera_->options->framerate = this->framerate_;
    this->camera_->options->shutter = this->shutter;
    this->camera_->options->verbose = this->verbose_;

    // get rostimestamp as variable init_camera_time
    // mesuring time to start camera
    this->camera_->options->setExposureMode(Exposure_Modes::EXPOSURE_SHORT);

    this->camera_->startVideo();

    camera_initilaize_time = this->now();

    this->image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 1);
    this->timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / this->framerate_), std::bind(&PiCameraROS::timerCallback, this));
}

PiCameraROS::~PiCameraROS()
{
    this->camera_->stopVideo();
    if (this->verbose_){
        cv::destroyAllWindows();
    }
}

void PiCameraROS::timerCallback()
{
    sensor_msgs::msg::Image image_msg;
    cv_bridge::CvImage cv_image;
    cv::Mat image;

    rclcpp::Time current_time = this->now();
    float time_diff = (current_time - camera_initilaize_time).seconds();

    // round to the camera FPS
    int frame_id = (int) (time_diff * (float)this->framerate_);

    float tmp_time = (float)frame_id / (float)this->framerate_;

    int time_s = (int) tmp_time;
    int time_ns = (int) ((tmp_time - (float)time_s) * 1e9);

    rclcpp::Time timestamp = rclcpp::Duration(time_s, time_ns) + camera_initilaize_time;

    this->camera_->getVideoFrame(image, 1000);

    // convert to grayscale
    cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);

    cv_image.encoding = sensor_msgs::image_encodings::MONO8;
    cv_image.image = image;
    cv_image.toImageMsg(image_msg);
    image_msg.header.frame_id = "camera";
    image_msg.header.stamp = timestamp;  
    image_pub_->publish(image_msg);

    // display image
    if (this->verbose_)
    {
        cv::imshow("Image", cv_image.image);
        cv::waitKey(1);
    }
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(picamera_ros::PiCameraROS)

int main(int argc, char** argv)
{
	using namespace picamera_ros;
    using namespace lccv;
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions options;
	auto node = std::make_shared<PiCameraROS>(options);

	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}