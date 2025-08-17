#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/msg/odometry.hpp>

using namespace std::chrono_literals;

class OffboardNode : public rclcpp::Node {
public:
  OffboardNode() : Node("offboard_node")
  {
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));
    offboard_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    traj_pub_          = create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    cmd_pub_           = create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
    tf_broadcaster_    = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    rclcpp::QoS qos_profile(10);
    qos_profile.best_effort(); 

    odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/fmu/out/vehicle_odometry", qos_profile,
      std::bind(&OffboardNode::odomCallback, this, std::placeholders::_1)
    );
    

    button_sub_ = create_subscription<std_msgs::msg::String>(
      "/button_cmd", 10,
      std::bind(&OffboardNode::buttonCallback, this, std::placeholders::_1) 
    );

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
  float target_z_ = -5.0f;
  float target_yaw = 0.0f;
  int counter_ = 0;

  void odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = this->get_clock()->now();
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = "base_link";

    tf_msg.transform.translation.x = msg->position[0];
    tf_msg.transform.translation.y = msg->position[1];
    tf_msg.transform.translation.z = -msg->position[2];

    tf2::Quaternion q_ned(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    tf2::Quaternion q_rotate;
    q_rotate.setRPY(M_PI, 0, 0);   

    tf2::Quaternion q_enu = q_rotate * q_ned;
    q_enu.normalize();

    tf_msg.transform.rotation.x = q_enu.x();
    tf_msg.transform.rotation.y = q_enu.y();
    tf_msg.transform.rotation.z = q_enu.z();
    tf_msg.transform.rotation.w = q_enu.w();

    tf_broadcaster_->sendTransform(tf_msg);

    geometry_msgs::msg::TransformStamped static_tf;
    static_tf.header.stamp = this->get_clock()->now();
    static_tf.header.frame_id = "base_link";
    static_tf.child_frame_id = "camera_link";

    static_tf.transform.translation.x = 0.0;
    static_tf.transform.translation.y = 0.0;
    static_tf.transform.translation.z = 0.0;

    tf2::Quaternion q_cam;
    q_cam.setRPY(0, 0, 0);  
    static_tf.transform.rotation.x = q_cam.x();
    static_tf.transform.rotation.y = q_cam.y();
    static_tf.transform.rotation.z = q_cam.z();
    static_tf.transform.rotation.w = q_cam.w();

    tf_broadcaster_->sendTransform(static_tf);

    geometry_msgs::msg::TransformStamped static_tf_optical;
    static_tf_optical.header.stamp = this->get_clock()->now();
    static_tf_optical.header.frame_id = "camera_link";
    static_tf_optical.child_frame_id = "camera_optical_frame";

    static_tf_optical.transform.translation.x = 0.0;
    static_tf_optical.transform.translation.y = 0.0;
    static_tf_optical.transform.translation.z = 0.0;

    tf2::Quaternion q_optical;
    q_optical.setRPY(0, 0, 0);  
    static_tf_optical.transform.rotation.x = q_optical.x();
    static_tf_optical.transform.rotation.y = q_optical.y();
    static_tf_optical.transform.rotation.z = q_optical.z();
    static_tf_optical.transform.rotation.w = q_optical.w();

    tf_broadcaster_->sendTransform(static_tf_optical);
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

  void publishTrajectorySetpoint(float x = 6.4f, float y = -9.98f , float z = -5.0f , float yaw = -0.90) {
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
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardNode>());
  rclcpp::shutdown();;
  return 0;
}
