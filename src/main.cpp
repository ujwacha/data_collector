#include "serial_class.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/subscription_base.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <chrono>
#include <functional>
#include <memory>

#define DEG2RAD 0.0174533f

char port_name[] =
  "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5XK3RJT-if00-port0";

#pragma pack(push, 1)

struct Twist_msg {
  float x;
  float y;
  float z;
};

struct ImuData {
  // data from imu
  float yaw;
  float pitch;
  float roll;
  float accel_x;
  float accel_y;
  float accel_z;
};

#pragma pack(pop)

namespace Serial_Bridge_Skeleton {
  class Sayer : public rclcpp::Node {
  private:
    SerialBridge bridge;
    // A bunch of Publishers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publ1;
    // A bunch of Subescribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      cmd_vel_subscriber;
    // Timer
    rclcpp::TimerBase::SharedPtr timer;

    // Send Timer
    rclcpp::TimerBase::SharedPtr sendtimer;
    rclcpp::TimerBase::SharedPtr pubtimer;

    // Global communicationdata to send to
    int edited = 0;
    Twist_msg glosend;

    ImuData c;
    int times = 0;

  public:
    Sayer(const rclcpp::NodeOptions &options) :
      rclcpp::Node("NodeName", options), bridge(port_name) {

      const rclcpp::QoS currentqol = rclcpp::QoS(10).best_effort();
      publ1 = this->create_publisher<sensor_msgs::msg::Imu>(
        "imu/data_raw", currentqol
      );
      cmd_vel_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel",
        currentqol,
        std::bind(
          &Sayer::twist_subscription_callback, this, std::placeholders::_1
        )
      );
      timer = this->create_wall_timer(
        std::chrono::milliseconds(3), std::bind(&Sayer::get_data, this)
      );

      pubtimer = this->create_wall_timer(
        std::chrono::milliseconds(10), std::bind(&Sayer::publish_data, this)
      );

      sendtimer = this->create_wall_timer(
        std::chrono::milliseconds(10), std::bind(&Sayer::send_data, this)
      );

      while (!bridge.init()) {
        RCLCPP_INFO(this->get_logger(), "Trying to open port: %s", port_name);
      }
    }

    void get_data() {

      ReadStatus read_status = bridge.receive_data((uint8_t *)&c, sizeof(c));
      switch (read_status) {
        case ReadStatus::READ_TIMEOUT:
          RCLCPP_ERROR(this->get_logger(), "Read Timeout");
          break;
        case ReadStatus::PORT_NOT_OPEN:
          RCLCPP_ERROR(this->get_logger(), "Port Not Open");
          bridge.init();
          break;
        case ReadStatus::START_BYTE_NOT_MATCHED:
          RCLCPP_ERROR(this->get_logger(), "Start Byte not matched");
          break;
        case ReadStatus::CRC_MISMATCH:
          RCLCPP_ERROR(this->get_logger(), "CRC Mismatch");
          break;
        case ReadStatus::DATA_RECEIVED_SUCCESSFULLY:
          RCLCPP_ERROR(this->get_logger(), "Read Timeout");
          publish_data();
          break;
        case ReadStatus::RUNTIME_ERROR:
          RCLCPP_ERROR(this->get_logger(), "Unexpected Error Occurred");
          bridge.port_close();
          bridge.init();
          break;
        default:
          RCLCPP_ERROR(this->get_logger(), "Unexpected Control Flow");
          break;
      }
    }

    void publish_data() {
      auto current_time = this->now();

      geometry_msgs::msg::Quaternion imu_quaternion;
      double cr = cos(c.roll * 0.5);
      double sr = sin(c.roll * 0.5);
      double cp = cos(c.pitch * 0.5);
      double sp = sin(c.pitch * 0.5);
      double cy = cos(c.yaw * 0.5);
      double sy = sin(c.yaw * 0.5);

      // Quaternion computation
      imu_quaternion.w = cr * cp * cy + sr * sp * sy;
      imu_quaternion.x = sr * cp * cy - cr * sp * sy;
      imu_quaternion.y = cr * sp * cy + sr * cp * sy;
      imu_quaternion.z = cr * cp * sy - sr * sp * cy;

      geometry_msgs::msg::Vector3 accel;
      accel.x = c.accel_x;
      accel.y = c.accel_y;
      accel.z = c.accel_z;

      auto msg = sensor_msgs::msg::Imu();

      msg.header.stamp = current_time;
      msg.header.frame_id = "imu";
      msg.orientation = imu_quaternion;
      msg.linear_acceleration = accel;
      msg.orientation_covariance = {1, 0, 0, 0, 1, 0, 0, 0, 1};
      msg.linear_acceleration_covariance = {1, 0, 0, 0, 1, 0, 0, 0, 1};
      times++;

      if (times == 10000) {
        times = 0;
        RCLCPP_INFO(this->get_logger(), "sent 10000 messages");
      }

      publ1->publish(msg);
    }

    void twist_subscription_callback(
      const geometry_msgs::msg::Twist::UniquePtr msg
    ) {

      RCLCPP_INFO(this->get_logger(), "sent 10000 messages");

      glosend.x = msg->linear.x;
      glosend.y = msg->linear.y;
      glosend.z = msg->angular.z;

      edited++;
    }

    void send_data() {
      // can be used if we want to make sure data is different
      // if (edited < 4) return;

      edited = 0;
      WriteStatus write_status =
        bridge.write_data((uint8_t *)&glosend, sizeof(glosend));

      switch (write_status) {
        case WriteStatus::PORT_NOT_OPEN:
          RCLCPP_ERROR(this->get_logger(), "Port not open");
          bridge.init();
          break;
        case WriteStatus::RUNTIME_ERROR:
          RCLCPP_ERROR(this->get_logger(), "Runtime Error");
          bridge.port_close();
          bridge.init();
          break;
        case WriteStatus::WRITEEN_SUCCESSFULLY:
          RCLCPP_INFO(this->get_logger(), "Sent Data Through Serial");
          break;
      }
    }
  };
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options = rclcpp::NodeOptions();
  rclcpp::spin(std::make_shared<Serial_Bridge_Skeleton::Sayer>(options));
  rclcpp::shutdown();

  return 0;
}
