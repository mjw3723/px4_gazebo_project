#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class OffboardNode : public rclcpp::Node {
public:
  OffboardNode() : Node("offboard_node")
  {
    offboard_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    traj_pub_          = create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    cmd_pub_           = create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
    rclcpp::QoS qos(rclcpp::KeepLast(5));
    qos.reliable();
    qos.durability_volatile();
    
    cam_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/realsense/camera/image_raw",   
        qos,
        std::bind(&OffboardNode::cameraCallback, this, std::placeholders::_1)
    );
    timer_ = create_wall_timer(50ms, std::bind(&OffboardNode::onTimer, this));
    arm();
    setOffboardMode();
  }

private:
  uint64_t nowUSec(){
    return this->get_clock()->now().nanoseconds() / 1000;
  }

  void arm() {
    sendCmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
  }

  void setOffboardMode() {
    sendCmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
  }

  void sendCmd(uint16_t cmd, float p1=0, float p2=0, float p3=0, float p4=0, float p5=0, float p6=0, float p7=0)
  {
    px4_msgs::msg::VehicleCommand m{};
    m.timestamp = nowUSec();
    m.param1=p1; m.param2=p2; m.param3=p3; m.param4=p4; m.param5=p5; m.param6=p6; m.param7=p7;
    m.command = cmd;
    m.target_system = 1;
    m.target_component = 1;
    m.source_system = 1;
    m.source_component = 1;
    m.from_external = true;
    cmd_pub_->publish(m);
  }
  void cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "Image received:");
        try {
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
            cv::imshow("PX4 Camera", frame);
            cv::waitKey(1);
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(get_logger(), "CV Bridge error: %s", e.what());
        }
    }


void onTimer()
{
    publishOffboardControlMode();
    publishTrajectorySetpoint();
    if (counter_ == 20) { 
        arm();
        setOffboardMode();
        RCLCPP_INFO(get_logger(), "ARM & Offboard mode enabled");
    }
    counter_++;
}

void publishOffboardControlMode() {
    px4_msgs::msg::OffboardControlMode mode{};
    mode.position = true;
    mode.timestamp = nowUSec();
    offboard_mode_pub_->publish(mode);
}

void publishTrajectorySetpoint() {
    px4_msgs::msg::TrajectorySetpoint sp{};
    sp.position = {0.0f, 0.0f, -2.0f};
    sp.yaw = 0.0f;
    sp.timestamp = nowUSec();
    traj_pub_->publish(sp);
}

int counter_ = 0;

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr  traj_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr      cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_sub_;  
};

int main(int argc, char** argv)
{
  cv::Mat img = cv::Mat::zeros(480, 640, CV_8UC3);
  cv::imshow("Test", img);
  cv::waitKey(0);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardNode>());
  rclcpp::shutdown();;
  return 0;
}
