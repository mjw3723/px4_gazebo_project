#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class OffboardNode : public rclcpp::Node {
public:
  OffboardNode() : Node("offboard_node")
  {
    offboard_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    traj_pub_          = create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    cmd_pub_           = create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

    pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", 10,
      std::bind(&OffboardNode::positionCallback, this, std::placeholders::_1)
    );

    button_sub_ = create_subscription<std_msgs::msg::String>(
      "/button_cmd", 10,
      std::bind(&OffboardNode::buttonCallback, this, std::placeholders::_1) 
    );

    rclcpp::QoS qos(rclcpp::KeepLast(5));
    qos.reliable();
    qos.durability_volatile();
    
    timer_ = create_wall_timer(50ms, std::bind(&OffboardNode::onTimer, this));
    arm();
    setOffboardMode();
  }

private:
  float curr_x_ = 0.0f;
  float curr_y_ = 0.0f;
  float curr_z_ = 0.0f;
  float target_x_ = 0.0f;
  float target_y_ = 0.0f;
  float target_z_ = -2.0f;
  float target_yaw = 0.0f;
  int counter_ = 0;

  void positionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    curr_x_ = msg->x;
    curr_y_ = msg->y;
    curr_z_ = msg->z;
  }

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

  void publishTrajectorySetpoint(float x = 0.0f, float y = 0.0f , float z = -2.0f , float yaw = 0.0) {
      px4_msgs::msg::TrajectorySetpoint sp{};
      sp.position = {target_x_, target_y_, target_z_};
      sp.yaw = target_yaw;  
      sp.timestamp = nowUSec();
      traj_pub_->publish(sp);
  }

  void buttonCallback(const std_msgs::msg::String::SharedPtr msg)
  {
      float step = 0.1f;
      if (msg->data == "up") {
        target_x_ += step * cos(target_yaw);
        target_y_ += step * sin(target_yaw);
      }
      else if (msg->data == "down") {
        target_x_ -= step * cos(target_yaw);
        target_y_ -= step * sin(target_yaw);
      }
      else if (msg->data == "left") {
        target_x_ += step * sin(target_yaw);
        target_y_ -= step * cos(target_yaw);
      }
      else if (msg->data == "right") {
        target_x_ -= step * sin(target_yaw);
        target_y_ += step * cos(target_yaw);
      }
      else if (msg->data == "yaw_left") {
        target_yaw -= 0.1f;
      }
      else if (msg->data == "yaw_right") {
        target_yaw += 0.1f;
      }
  }
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr  traj_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr      cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr button_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pos_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardNode>());
  rclcpp::shutdown();;
  return 0;
}
