#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
//#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/subscription_base.hpp>
#include <rclcpp/timer.hpp>
#define DEG2RAD 0.0174533f

#include <rclcpp/utilities.hpp>
// #include <string>
#include "serial_class.hpp"

#include "rclcpp_components/register_node_macro.hpp"
namespace Serial_Bridge_Skeleton {
  class Sayer : public rclcpp::Node {

    int times = 0;

  public:
    Sayer(const rclcpp::NodeOptions &options)
      : rclcpp::Node("NodeName", options), bridge("/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5XK3RJT-if00-port0") {

      const rclcpp::QoS currentqol = rclcpp::QoS(10).best_effort();

      publ1 =
	this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", currentqol);
      // publ2 = this->create_publisher<geometry_msgs::msg::Vector3>("publ2",
      //                                                             currentqol);
      // publ3 =
      //     this->create_publisher<geometry_msgs::msg::Twist>("publ3", currentqol);
      // boolpubl = this->create_publisher<std_msgs::msg::Bool>("boolpublisher",
      //                                                        currentqol);

      cmd_vel_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
										"cmd_vel", currentqol,
										std::bind(&Sayer::twist_subscription_callback, this, std::placeholders::_1));

      // subscription2 = this->create_subscription<geometry_msgs::msg::Vector3>(
      //     "sub2", currentqol,
      //     std::bind(&Sayer::sub2fun, this, std::placeholders::_1));
      //
      // subscription3 = this->create_subscription<geometry_msgs::msg::Twist>(
      //     "sub3", currentqol,
      //     std::bind(&Sayer::sub3fun, this, std::placeholders::_1));
      //
      // boolsub = this->create_subscription<std_msgs::msg::Bool>(
      //     "boolsub", currentqol,
      //     std::bind(&Sayer::boolsubfun, this, std::placeholders::_1));
      //
      timer = this->create_wall_timer(std::chrono::milliseconds(3),
				      std::bind(&Sayer::get_data, this));

      pubtimer = this->create_wall_timer(std::chrono::milliseconds(10),
					 std::bind(&Sayer::publish_data, this));

      sendtimer = this->create_wall_timer(std::chrono::milliseconds(10),
       					  std::bind(&Sayer::send_data, this));

    }

    void get_data() {

      // return; // don't want to recieve data for now

      try {
	c = bridge.attempt_get_non_blocking();
      } catch (int e) {
	switch (e) {
	case -1:
	  RCLCPP_ERROR(this->get_logger(), "Start Byte Search Ongoing");
	  break;

	case -2:
	  RCLCPP_ERROR(this->get_logger(), "Corrupt Data Found");
	  break;

	default:
	  RCLCPP_ERROR(this->get_logger(), "Some Random Error Thrown, Code: %i",
		       e);
	}
	return;
      }
      // print_imu_data(c);

      //			 publish_data(c);
    }

    void publish_data() {
      publish_imu_data(c);
    }

    void publish_imu_data(ImuData c) {
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

      //			imu_quaternion.setRPY(c.roll * DEG2RAD,c.pitch * DEG2RAD,c.yaw * DEG2RAD);

      geometry_msgs::msg::Vector3 accel;
      accel.x = c.accel_x;
      accel.y = c.accel_y;
      accel.z = c.accel_z;


      auto msg = sensor_msgs::msg::Imu();

			
      msg.header.stamp = current_time;
      msg.header.frame_id = "imu";
	

      msg.orientation = imu_quaternion;


      msg.linear_acceleration = accel;

      msg.orientation_covariance = {1, 0, 0,
				    0, 1, 0,
				    0, 0, 1};

      msg.linear_acceleration_covariance = {1, 0, 0,
					    0, 1, 0,
					    0, 0, 1};


      // message.data = "Hello";
      times++;
      if (times == 10000) {
	times = 0;
	RCLCPP_INFO(this->get_logger(), "sent 10000 messages");
      }

      publ1->publish(msg);
      // publ2->publish(message2);
      // publ3->publish(message3);
      //
      // boolpubl->publish(boolmsg);
    }

    void twist_subscription_callback(const geometry_msgs::msg::Twist::UniquePtr msg) {

      RCLCPP_INFO(this->get_logger(), "sent 10000 messages");

      glosend.x = msg->linear.x;
      glosend.y = msg->linear.y;
      glosend.omega = msg->angular.z;

      edited++;
    }

    // void sub2fun(const geometry_msgs::msg::Vector3::UniquePtr msg) {
    //   glosend.x = msg->x;
    //   glosend.y = msg->y;
    //   glosend.omega = msg->z;
    //
    //   edited++;
    // }
    //
    // void sub3fun(const geometry_msgs::msg::Twist::UniquePtr msg) {
    //   // glosend.zz = msg->linear.z;
    //   // glosend.theta = msg->angular.z;
    //
    //   // edited++;
    // }
    //
    // void boolsubfun(const std_msgs::msg::Bool::UniquePtr msg) {
    //   // glosend.on_off = msg->data;
    //
    //   // edited++;
    // };
    //
    void send_data() {

      // can be used if we want to make sure data is different
      // if (edited < 4) return;

      edited = 0;

      make_sendable_with_metadata(glosend);
      try {
	if (bridge.attempt_send_probably_blocking(glosend)) {


	  RCLCPP_INFO(this->get_logger(), "Sent Data Through Serial");
	  
	} // it will always return 1 as it is turned into sendable data already, no it won't because of another state in the system


      } catch (int i) {
	switch (i) {
	case -1:
	  RCLCPP_ERROR(this->get_logger(), "Couldn't Send Data");
	  break;
	default:
	  RCLCPP_ERROR(this->get_logger(), "Unknown Error Occoured While Sending Data");
	  break;
	}
      }
    }

  private:
    BridgeClass bridge;
    // A bunch of Publishers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publ1;
    // rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publ2;
    // rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publ3;
    //
    // rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr boolpubl;

    // A bunch of Subescribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber;
    // rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscription2;
    // rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription3;
    //
    // rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriptionlol;
    //
    // rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr boolsub;


    // rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr anotherstupidbool; 




    // Timer
    rclcpp::TimerBase::SharedPtr timer;

    // Send Timer
    rclcpp::TimerBase::SharedPtr sendtimer;
    rclcpp::TimerBase::SharedPtr pubtimer;

    // Global communicationdata to send to
    int edited = 0;
    Twist_msg glosend;

	  
    ImuData c;
  };

} //namespace //Serial_Bridge_Skeleton;

// RCLCPP_COMPONENTS_REGISTER_NODE(Serial_Bridge_Skeleton::Sayer);

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options = rclcpp::NodeOptions();

  rclcpp::spin(std::make_shared<Serial_Bridge_Skeleton::Sayer>(options));

  rclcpp::shutdown();

  return 0;
}
