#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
// #include "turtlelib/diff_drive.hpp"
#include <catch2/catch_test_macros.hpp>
#include<iostream>
using namespace std::chrono_literals;

nuturtlebot_msgs::msg::WheelCommands wheel_cmd;
sensor_msgs::msg::JointState joint_state;
bool got_wheel_cmd = false;
void sub_callback(const nuturtlebot_msgs::msg::WheelCommands::SharedPtr msg)
{
  wheel_cmd.left_velocity = msg->left_velocity;
  wheel_cmd.right_velocity = msg->right_velocity;
  got_wheel_cmd = true;
}

void encoder_sub(sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::cout << "Encoder data received" << std::endl;
  std::cout << "Left wheel velocity: " << msg->velocity[0] << std::endl;
  
  joint_state.position = msg->position;
  joint_state.velocity = msg->velocity;
  got_wheel_cmd = true;
}

TEST_CASE("turtle_control_test", "[pure_translation]") {

  auto node = rclcpp::Node::make_shared("turtle_control_test");
  auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  auto subscriber = node->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
    "wheel_cmd", 10,
    &sub_callback);
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 0.01;
  twist.angular.z = 0.0;

  while (
    rclcpp::ok() && !got_wheel_cmd
  )
  {
    publisher->publish(twist);

    rclcpp::spin_some(node);
  }

  CHECK(got_wheel_cmd);
  REQUIRE_THAT(wheel_cmd.left_velocity, Catch::Matchers::WithinAbs(12, 0.01));
  REQUIRE_THAT(wheel_cmd.right_velocity, Catch::Matchers::WithinAbs(12, 0.01));
}
TEST_CASE("turtle_control_test", "[pure_rotation]") {
  got_wheel_cmd = false;
  auto node = rclcpp::Node::make_shared("turtle_control_test");
  auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  auto subscriber = node->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
    "wheel_cmd", 10,
    &sub_callback);
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 0.0;
  twist.angular.z = 0.1;

  while (
    rclcpp::ok() && !got_wheel_cmd
  )
  {
    publisher->publish(twist);

    rclcpp::spin_some(node);
  }

  CHECK(got_wheel_cmd);
  REQUIRE_THAT(wheel_cmd.left_velocity, Catch::Matchers::WithinAbs(-10, 0.01));
  REQUIRE_THAT(wheel_cmd.right_velocity, Catch::Matchers::WithinAbs(10, 0.01));
}
TEST_CASE("turtle_control_test","[encoder_test]") {
  got_wheel_cmd = false;
  auto node = rclcpp::Node::make_shared("turtle_control_test");
  auto publisher = node->create_publisher<nuturtlebot_msgs::msg::SensorData>("sensor_data", 10);
  auto subscriber = node->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10,&encoder_sub);
  

  nuturtlebot_msgs::msg::SensorData sensor_data;
  sensor_data.left_encoder = 100;
  sensor_data.right_encoder = 100;
  sensor_data.stamp = node->get_clock()->now();
  std::cout<<"Publishing sensor data"<<std::endl;
  while (
    rclcpp::ok() && !got_wheel_cmd
  )
  {
    sensor_data.stamp = node->get_clock()->now();
    publisher->publish(sensor_data);

    rclcpp::spin_some(node);
  }

  CHECK(got_wheel_cmd);
  REQUIRE_THAT(joint_state.position[0], Catch::Matchers::WithinAbs(0.15, 0.01));
  REQUIRE_THAT(joint_state.position[1], Catch::Matchers::WithinAbs(0.15, 0.01));
  REQUIRE_THAT(joint_state.velocity[0], Catch::Matchers::WithinAbs(0, 0.01));
  REQUIRE_THAT(joint_state.velocity[1], Catch::Matchers::WithinAbs(0, 0.01));


}