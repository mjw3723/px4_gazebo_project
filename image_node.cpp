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
                hsv_center_yellow_mask(image);
            }
            catch (cv_bridge::Exception &e) {
                std::cerr << "cv_bridge 변환 실패: " << e.what() << std::endl;
            }
        }

        void hsv_center_yellow_mask(const cv::Mat& bgr_image)
        {
            // 1. HSV 변환
            cv::Mat hsv;
            cv::cvtColor(bgr_image, hsv, cv::COLOR_BGR2HSV);

            // 2. 노란색 범위 설정
            cv::Scalar lower_yellow(20, 100, 100);
            cv::Scalar upper_yellow(30, 255, 255);
            cv::Mat mask;
            cv::inRange(hsv, lower_yellow, upper_yellow, mask);

            // 3. ROI 설정 (중앙 기준 상하 절반만 보기)
            int width = mask.cols;
            int height = mask.rows;
            int roi_y = height / 2;
            cv::Mat roi = mask(cv::Rect(0, roi_y, width, height - roi_y));  // 하단 절반만 사용

            // 4. 윤곽선 검출
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(roi, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            // 5. 중앙과 가장 가까운 좌우 2개 선 찾기
            int center_x = width / 2;
            std::vector<std::pair<int, std::vector<cv::Point>>> contour_distances;

            for (const auto& contour : contours) {
                cv::Rect bound = cv::boundingRect(contour);
                int cx = bound.x + bound.width / 2;
                int dist = std::abs(cx - center_x);
                contour_distances.emplace_back(dist, contour);
            }

            // 거리 기준 정렬
            std::sort(contour_distances.begin(), contour_distances.end(),
                    [](auto& a, auto& b) { return a.first < b.first; });

            // 6. 가장 가까운 2개만 마스크에 그리기
            cv::Mat result_mask = cv::Mat::zeros(mask.size(), CV_8UC1);
            for (size_t i = 0; i < std::min(size_t(2), contour_distances.size()); ++i) {
                cv::drawContours(result_mask(cv::Rect(0, roi_y, width, height - roi_y)),
                                std::vector<std::vector<cv::Point>>{contour_distances[i].second},
                                -1, 255, cv::FILLED);
            }

            cv::imshow("Yellow Mask", mask);
            cv::imshow("Closest 2 Lines", result_mask);
            cv::waitKey(1);
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