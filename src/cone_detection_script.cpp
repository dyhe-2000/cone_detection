#include <iostream>
#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include <cv_bridge/cv_bridge.h>
using std::placeholders::_1;

class ConeDetector : public rclcpp::Node{
public:
	ConeDetector() : Node("cone_detector"){
		this->img_sub_ = this->create_subscription<sensor_msgs::msg::Image>("image_raw", 10, std::bind(&ConeDetector::topic_callback, this, _1));
		this->img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("cone_detection", 10);
		this->pos_pixels_over_thresh_pub_ = this->create_publisher<std_msgs::msg::Bool>("pos_pixels_over_thresh", 10);
	}
private:
	void topic_callback(const sensor_msgs::msg::Image& msg) const {
		std::cout << "received an Image" << std::endl;
		int img_height =  msg.height;
		int img_width = msg.width;

		cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(msg, msg.encoding);
		// std::cout << "msg encoding: " << msg.encoding << std::endl; // bgr8
		// std::cout << "img encoding: " << img_ptr->encoding << std::endl;

		cv::Mat frame = img_ptr->image;
		// std::cout << frame.rows << std::endl;
		// std::cout << frame.cols << std::endl;
		// std::cout << frame.channels() << std::endl;
		// cv::imwrite("/home/jetson/dev_ws/image.jpg",frame);
		
		// set single channel and binary image
		cv::Mat img_0_1;
		frame.convertTo(img_0_1, CV_32FC3, 1.f/255);

		cv::Mat bgr[3]; // splitting channels from original image
		cv::Mat single_channel_image(img_height, img_width, CV_32FC1, cv::Scalar(0));
		// cv::Mat binary_image(img_height, img_width, CV_8UC1, cv::Scalar(0));

		cv::split(img_0_1,bgr);
		single_channel_image = -2954.6507578 * bgr[0] * 255 + 
				       -1753.88636375 * bgr[1] * 255 + 
				       3325.96402434 * bgr[2] * 255;
		cv::threshold(single_channel_image, single_channel_image,
				190000.0, 255.0, 0);
		single_channel_image.convertTo(single_channel_image, CV_8UC1);

		uint8_t* single_channel_pixel_ptr = (uint8_t*)single_channel_image.data;
		int pos_pixels_count = 0;
		for(int i = 0; i < frame.rows; i++){
			for(int j = 0; j < frame.cols; j++){
				if(single_channel_pixel_ptr[i*frame.cols + j] == 255){
					pos_pixels_count += 1;
				}
			}
		}
		// std::cout << "pos_pixels_count: " << pos_pixels_count << std::endl;
		// lower 650 upper 20500
		auto message = std_msgs::msg::Bool();
		if(650 < pos_pixels_count && pos_pixels_count < 20500){
			message.data = true;
		}
		else{
			message.data = false;
		}
		this->pos_pixels_over_thresh_pub_->publish(message);
			
		/*
		uint8_t* original_pixel_ptr = (uint8_t*)frame.data;
		uint8_t* single_channel_pixel_ptr = (uint8_t*)single_channel_image.data;
		int cn = frame.channels(); // original img channel number
		cv::Scalar_<uint8_t> original_bgr_pixel;

		for(int i = 0; i < frame.rows; i++){
			for(int j = 0; j < frame.cols; j++){
				original_bgr_pixel.val[0] = 
					original_pixel_ptr[i*frame.cols*cn + j*cn + 0]; // B
				original_bgr_pixel.val[1] = 
					original_pixel_ptr[i*frame.cols*cn + j*cn + 1]; // G
				original_bgr_pixel.val[2] = 
					original_pixel_ptr[i*frame.cols*cn + j*cn + 2]; // R
				// do something with BGR values...
				single_channel_pixel_ptr[i*frame.cols + j] = 
					0.299 * original_bgr_pixel.val[2] + 
					0.587 * original_bgr_pixel.val[1] + 
					0.114 * original_bgr_pixel.val[0];
			}
		}
		*/

		// bounding box image
		// int x = 300;
		// int y = 300;
		// cv::Point pt1(x, y);
		// cv::Point pt2(img_width-300, img_height-300);
		// cv::rectangle(single_channel_image, pt1, pt2, cv::Scalar(255));

		// Avoid copying image message if possible
		sensor_msgs::msg::Image::UniquePtr image_msg(new sensor_msgs::msg::Image());
		auto stamp = now();
		// Convert OpenCV Mat to ROS Image
		image_msg->header.stamp = stamp;
		// image_msg->header.frame_id = cxt_.camera_frame_id_;
		image_msg->height = frame.rows;
		image_msg->width = frame.cols;
		image_msg->encoding = "8UC1";
		image_msg->is_bigendian = false;
		image_msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(
				single_channel_image.step);
		image_msg->data.assign(single_channel_image.datastart, 
					single_channel_image.dataend);
		img_pub_->publish(std::move(image_msg));
		return;
	}
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pos_pixels_over_thresh_pub_;
};

int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ConeDetector>());
	rclcpp::shutdown();
	return 0;
}
