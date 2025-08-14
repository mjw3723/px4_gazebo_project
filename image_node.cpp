#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class ImageNode : public rclcpp::Node {
    public:
        ImageNode() : Node("image_node")
        { 
            rgb_camera_sub_ = create_subscription<sensor_msgs::msg::Image>(
                "/realsense/camera/image_raw",
                10,
                std::bind(&ImageNode::rgb_callback,this,std::placeholders::_1)
            );

        }
    private:
        void rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg){
            try {
                cv::Mat hsv_image;
                cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;
                hsv_image = hsv_cvt(image);
                cv::imshow("hsv_image ", hsv_image);
                cv::waitKey(1);  
            }
            catch (cv_bridge::Exception &e) {
                std::cerr << "cv_bridge 변환 실패: " << e.what() << std::endl;
            }
        }

        cv::Mat hsv_cvt(const cv::Mat& bgr_image){
            cv::Mat hsv_image;
            cv::cvtColor(bgr_image, hsv_image, cv::COLOR_BGR2HSV);
            
            cv::Scalar lower_yellow(20,100,100);
            cv::Scalar upper_yellow(20,255,255);

            cv::Mat yellow_mask;
            cv::inRange(hsv_image, lower_yellow, upper_yellow , yellow_mask);
            return hsv_image;
        }

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_camera_sub_;

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageNode>());
    rclcpp::shutdown();;
    return 0;
}