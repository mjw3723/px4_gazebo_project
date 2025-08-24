#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/msg/odometry.hpp>


using namespace std::chrono_literals;

class SensorNode : public rclcpp::Node {
public:
    SensorNode() : Node("SensorNode"){
        this->set_parameter(rclcpp::Parameter("use_sim_time", true));
        tf_broadcaster_    = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        rclcpp::QoS qos_profile(10);
        qos_profile.best_effort(); 

        odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos_profile,
            std::bind(&SensorNode::odomCallback, this, std::placeholders::_1)
        );
        sensor_sub_ = this->create_subscription<px4_msgs::msg::SensorCombined>(
            "/fmu/out/sensor_combined",
            qos_profile,
            std::bind(&SensorNode::sensorCallback, this, std::placeholders::_1)
        );
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
        static_tf_pub();
    }   

private:
    void static_tf_pub(){
        geometry_msgs::msg::TransformStamped static_tf;
        static_tf.header.stamp = rclcpp::Time(0);
        static_tf.header.frame_id = "base_link";
        static_tf.child_frame_id = "camera_link";
        static_tf.transform.translation.x = 0.0;
        static_tf.transform.translation.y = 0.0;
        static_tf.transform.translation.z = 0.0;
        tf2::Quaternion q_cam;
        q_cam.setRPY(0,0,0);
        static_tf.transform.rotation.x = q_cam.x();
        static_tf.transform.rotation.y = q_cam.y();
        static_tf.transform.rotation.z = q_cam.z();
        static_tf.transform.rotation.w = q_cam.w();
        static_broadcaster_->sendTransform(static_tf);

        // camera_link -> camera_optical_frame
        geometry_msgs::msg::TransformStamped static_tf_optical;
        static_tf_optical.header.stamp = rclcpp::Time(0);
        static_tf_optical.header.frame_id = "camera_link";
        static_tf_optical.child_frame_id = "camera_optical_frame";
        static_tf_optical.transform.translation.x = 0.0;
        static_tf_optical.transform.translation.y = 0.0;
        static_tf_optical.transform.translation.z = 0.0;
        tf2::Quaternion q_optical;
        q_optical.setRPY(-M_PI/2, 0, -M_PI/2);  // optical frame 규약
        static_tf_optical.transform.rotation.x = q_optical.x();
        static_tf_optical.transform.rotation.y = q_optical.y();
        static_tf_optical.transform.rotation.z = q_optical.z();
        static_tf_optical.transform.rotation.w = q_optical.w();
        static_broadcaster_->sendTransform(static_tf_optical);
    }

    void odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg){
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
    }

    void sensorCallback(const px4_msgs::msg::SensorCombined::SharedPtr msg)
    {
        sensor_msgs::msg::Imu imu_msg;

        // Header
        imu_msg.header.stamp = this->get_clock()->now();
        imu_msg.header.frame_id = "base_link"; // 필요하면 "imu_link" 등으로 변경

        // Gyro (rad/s)
        imu_msg.angular_velocity.x = msg->gyro_rad[0];
        imu_msg.angular_velocity.y = msg->gyro_rad[1];
        imu_msg.angular_velocity.z = msg->gyro_rad[2];

        // Accel (m/s^2)
        imu_msg.linear_acceleration.x = msg->accelerometer_m_s2[0];
        imu_msg.linear_acceleration.y = msg->accelerometer_m_s2[1];
        imu_msg.linear_acceleration.z = msg->accelerometer_m_s2[2];

        // Orientation은 PX4에서 직접 제공되지 않음 → 기본값 NaN (or 0)
        imu_msg.orientation.x = 0.0;
        imu_msg.orientation.y = 0.0;
        imu_msg.orientation.z = 0.0;
        imu_msg.orientation.w = 1.0;

        // Covariance (임시로 -1 → unknown)
        for (int i = 0; i < 9; i++) {
            imu_msg.angular_velocity_covariance[i] = 0.0;
            imu_msg.linear_acceleration_covariance[i] = 0.0;
            imu_msg.orientation_covariance[i] = -1.0;
        }

        // Publish
        imu_pub_->publish(imu_msg);
    }

    uint64_t nowUSec(){
        return this->get_clock()->now().nanoseconds() / 1000;
    }


    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
    rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr sensor_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorNode>());
    rclcpp::shutdown();;
    return 0;
}
