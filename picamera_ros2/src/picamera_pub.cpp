#include "picamera_ros2/picamera_pub.hpp"

namespace picamera_ros
{

PiCameraROS::PiCameraROS(const rclcpp::NodeOptions &options_): Node("picamera_ros", options_)
{
    this->camera_ = new lccv::PiCamera();

    this->param_listener_ = std::make_shared<picamera_parameters::ParamListener>(
        this->get_node_parameters_interface());
    this->params_ = param_listener_->get_params();

    std::map<std::string, int> exposure_map =
		{ { "normal", libcamera::controls::ExposureNormal },
        { "sport", libcamera::controls::ExposureShort },
        { "long", libcamera::controls::ExposureLong },
        { "custom", libcamera::controls::ExposureCustom } };
    std::map<std::string, int> awb_map =
        { { "auto", libcamera::controls::AwbAuto },
        { "incandescent", libcamera::controls::AwbIncandescent },
        { "tungsten", libcamera::controls::AwbTungsten },
        { "fluorescent", libcamera::controls::AwbFluorescent },
        { "indoor", libcamera::controls::AwbIndoor },
        { "daylight", libcamera::controls::AwbDaylight },
        { "cloudy", libcamera::controls::AwbCloudy },
        { "custom", libcamera::controls::AwbCustom } };
    std::map<std::string, int> metering_table =
		{ { "centre", libcamera::controls::MeteringCentreWeighted },
        { "spot", libcamera::controls::MeteringSpot },
        { "average", libcamera::controls::MeteringMatrix },
        { "matrix", libcamera::controls::MeteringMatrix },
        { "custom", libcamera::controls::MeteringCustom } };

    this->camera_->options->video_width = this->params_.video_width;
    this->camera_->options->video_height = this->params_.video_height;
    this->camera_->options->photo_width = this->params_.camera_width;
    this->camera_->options->photo_height = this->params_.camera_height;
    this->camera_->options->camera = this->params_.id;
    this->camera_->options->shutter = this->params_.shutter;
    this->camera_->options->denoise = this->params_.denoise;
    this->camera_->options->brightness = this->params_.brightness;
    this->camera_->options->saturation = this->params_.saturation;
    this->camera_->options->ev = this->params_.ev;
    this->camera_->options->sharpness = this->params_.sharpness;
    this->camera_->options->contrast = this->params_.contrast;
    this->camera_->options->framerate = this->params_.framerate;
    this->camera_->options->roi_x = this->params_.autofocus_range.x;
    this->camera_->options->roi_y = this->params_.autofocus_range.y;
    this->camera_->options->roi_width = this->params_.autofocus_range.width;
    this->camera_->options->roi_height = this->params_.autofocus_range.height;
    this->camera_->options->setMetering(static_cast<Metering_Modes>(metering_table[this->params_.metering]));
    this->camera_->options->setExposureMode(static_cast<Exposure_Modes>(exposure_map[this->params_.exposure]));
    this->camera_->options->setWhiteBalance(static_cast<WhiteBalance_Modes>(awb_map[this->params_.awb]));

    this->camera_->options->framerate = (float)this->params_.framerate;

    this->camera_->hdrOpen(this->params_.hdr);
    this->camera_->startVideo();

    this->image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);
    this->timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / this->params_.framerate), std::bind(&PiCameraROS::timerCallback, this));
}

PiCameraROS::~PiCameraROS()
{
    this->camera_->stopVideo();
}

void PiCameraROS::timerCallback()
{
    sensor_msgs::msg::Image image_msg;
    cv_bridge::CvImage cv_image;
    cv::Mat image;

    cv_image.encoding = sensor_msgs::image_encodings::BGR8;
    this->camera_->getVideoFrame(image, 1000);
    cv_image.image = image;
    cv_image.toImageMsg(image_msg);
    image_pub_->publish(image_msg);
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