#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
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
#include "sick_interfaces/msg/sick.hpp"
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>

struct Twist_msg {
  float x;
  float y;
  float z;
};

struct ImuData {
  double yaw;
  double pitch;
  double roll;
  float accel_x;
  float accel_y;
  float accel_z;
};

struct OdomMsg {
  // float x;
  // float y;
  // float theta;
  // float vx;
  // float vy;
  // float w;
  float omega_l;
  float omega_r;
  float omega_m;
};

struct Distance_Sensors {
  float d_mid;
  uint16_t s_mid;
  float d_right;
  uint16_t s_right;
  float d_left;
  uint16_t s_left;
};

struct SickData {
  float d_one;
  bool works_one;
  float d_two;
  bool works_two;
  float d_three;
  bool works_three;
  float d_four;
  bool works_four;
}


Twist_msg twist_msg;
ImuData imu_data;
OdomMsg odom_msg;
Distance_Sensors dis_mrl;
SickData sick;
bool sick_update = false;

class Sayer : public rclcpp::Node {
private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr odom_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr rpy_sub;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr distance_sub;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr strength_sub;
  rclcpp::Subscription<sick_interfaces::msg::Sick>::SharedPtr sick_sub;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Time time;
  double initial_time;

public:
  Sayer(const rclcpp::NodeOptions &options) : Node("sayer_node", options) {
    const rclcpp::QoS currentqol = rclcpp::QoS(10).best_effort().durability_volatile();

    cmd_vel_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel",
      currentqol,
      std::bind(
        &Sayer::cmd_vel_subscription_callback, this, std::placeholders::_1
      )
    );


    rpy_sub = this->create_subscription<geometry_msgs::msg::Vector3>(
      "imu/rpy",
      currentqol,
      std::bind(
        &Sayer::rpy_callback, this, std::placeholders::_1
      )
    );


    distance_sub = this->create_subscription<geometry_msgs::msg::Vector3>(
      "/distance_mrl/length",
      currentqol,
      std::bind(
        &Sayer::distance_callback, this, std::placeholders::_1
      )
    );

    strength_sub = this->create_subscription<geometry_msgs::msg::Vector3>(
      "/distance_mrl/strength",
      currentqol,
      std::bind(
        &Sayer::strength_callback, this, std::placeholders::_1
      )
    );




    imu_subscriber = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data_raw",
      currentqol,
      std::bind(&Sayer::imu_subscription_callback, this, std::placeholders::_1)
    );

    odom_subscriber = this->create_subscription<geometry_msgs::msg::Vector3>(
      "odom_vec",
      currentqol,
      std::bind(&Sayer::odom_subscription_callback, this, std::placeholders::_1)
    );

    sick_sub = this->create_subscription<sick_interfaces::msg::Sick>(
      "sick/data",
      currentqol,
      std::bind(&Sayer::sick_sub_callback, this, std::placeholders::_1)
    );



    timer = this->create_wall_timer(
      std::chrono::milliseconds(10), std::bind(&Sayer::print_data, this)
    );

    initial_time = this->now().seconds();
    // initial_time = time.n();
  }

  void cmd_vel_subscription_callback(
    const geometry_msgs::msg::Twist::SharedPtr msg
  ) {
    twist_msg.x = msg->linear.x;
    twist_msg.y = msg->linear.y;
    twist_msg.z = msg->angular.z;
  }

  void rpy_callback(
    const geometry_msgs::msg::Vector3::SharedPtr msg
  ) {
    imu_data.roll = msg->x;
    imu_data.pitch = msg->y;
    imu_data.yaw = msg->z;
  }

  void distance_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
    dis_mrl.d_mid = msg->x;
    dis_mrl.d_right = msg->y;
    dis_mrl.d_left = msg->z;
  }

void strength_callback(
    const geometry_msgs::msg::Vector3::SharedPtr msg
  ) {
    dis_mrl.s_mid = msg->x;
    dis_mrl.s_right = msg->y;
    dis_mrl.s_left = msg->z;
  }


  void imu_subscription_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    const auto &orientation = msg->orientation;

    tf2::Quaternion quat(
      orientation.x, orientation.y, orientation.z, orientation.w
    );

//    tf2::Matrix3x3(quat).getRPY(imu_data.roll, imu_data.pitch, imu_data.yaw);

    const auto &acceleration = msg->linear_acceleration;
    imu_data.accel_x = acceleration.x;
    imu_data.accel_y = acceleration.y;
    imu_data.accel_z = acceleration.z;
  }

  void odom_subscription_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
    odom_msg.omega_l = msg->x;
    odom_msg.omega_r = msg->y;
    odom_msg.omega_m = msg->z;
  }

  void sick_callback(const sick_interfaces::msg::Sick::SharedPtr msg) {
    sick.d_one = msg.distance_one;
    sick.d_two = msg.distance_two;
    sick.d_three = msg.distance_three;
    sick.d_four = msg.distance_four;
    sick.works_one = msg.works_one;
    sick.works_two = msg.works_two;
    sick.works_three = msg.works_three;
    sick.works_four = msg.works_four;
    sick_update = true;
    print_data();
    sick_update = false;
  }
  void print_data() {
    auto current_time = this->now();

    double time_elapsed = this->now().seconds() - initial_time;
    std::cout << time_elapsed << ", " << twist_msg.x << "," << twist_msg.y
              << "," << twist_msg.z << ", " << odom_msg.omega_l << "," << odom_msg.omega_r
              << "," << odom_msg.omega_m << "," << odom_msg.omega_l<< ","
              << odom_msg.omega_r<< "," << odom_msg.omega_m << ", " << imu_data.roll
              << "," << imu_data.pitch << "," << imu_data.yaw << ","
              << imu_data.accel_x << "," << imu_data.accel_y << "," << imu_data.accel_z
	      << "," << dis_mrl.d_mid << "," << dis_mrl.d_right << "," << dis_mrl.d_left 
	      << "," << dis_mrl.s_mid << "," << dis_mrl.s_right << "," << dis_mrl.s_left
	      << "," << sick.d_one << "," << sick.d_two << "," << sick.d_three << "," << sick.d_four
	      << "," << sick.works_one << "," << sick.works_two << "," << sick.works_three << "," << sick.works_four
	      << "," << sick_update
	      << std::endl;
  }

};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  std::cout
    << "time stamp, twistx, twisty, twistz, omega_l, omega_r, omega_m, omega_l, omega_r, omega_m, roll, pitch, yaw, accl_x, accl_y, accl_z, dis_m, dis_r, dis_l, str_m, str_r, str_l, sick_one, sick_two, sick_three, sick_four, works_one, works_two, works_three, works_four, sick_update\n";
  rclcpp::spin(std::make_shared<Sayer>(options));
  rclcpp::shutdown();
  return 0;
}
