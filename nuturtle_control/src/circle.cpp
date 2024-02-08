#include "rclcpp/rclcpp.hpp"
#include <string>
#include <cmath>
#include "yaml-cpp/yaml.h"
#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <map>

#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/empty.hpp"

#include "turtlelib/diff_drive.hpp"

#include "nuturtle_control/srv/control.hpp"

using std::string;

using namespace std::chrono_literals;
using namespace turtlelib;

using std::placeholders::_1;
using std::placeholders::_2;

class CircleNode : public rclcpp::Node
{
public:
  //
  // CONSTRUCTOR
  //
  CircleNode() : Node("circle"),
                  is_reverse(false),
                  linear_vel(0.0),
                  angular_vel(0.0),
                  pub_active(false),
                  last_pub(false)
  {
    //
    // PARAMETERS
    //
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{}; // Prepare for parameter descriptions

    param_desc.description = "Frequency of node timer";
    declare_parameter("frequency", 100.0, param_desc);
    frequency = get_parameter("frequency").as_double();

    //
    // Additional variable initialization
    //
    init_var();

    //
    // SUBSCRIBERS
    //

    //
    // PUBLISHERS
    //
    cmd_vel_pub = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 100);

    //
    // SERVICE
    //
    control_srv = create_service<nuturtle_control::srv::Control>("control", std::bind(&CircleNode::control_callback, this, _1, _2));
    reverse_srv = create_service<std_srvs::srv::Empty>("reverse", std::bind(&CircleNode::reverse_callback, this, _1, _2));
    stop_srv = create_service<std_srvs::srv::Empty>("stop", std::bind(&CircleNode::reverse_callback, this, _1, _2));
    //
    // BROADCASTER
    //

    //
    // TIMER
    //
    timer = create_wall_timer(1.0s / frequency, std::bind(&CircleNode::timer_callback, this));
  }

private:
  //
  // Node-related Declarations
  //
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
  rclcpp::Service<nuturtle_control::srv::Control>::SharedPtr control_srv;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reverse_srv;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_srv;

  //
  // Variables
  //
  double frequency;
  bool is_reverse;
  double linear_vel, angular_vel;
  bool pub_active, last_pub;

  //
  // Objects
  //

  //
  // TIMER CALLBACK
  //
  /// @brief Timer callback for node, reads joy_state to publish appropriate output messages
  void timer_callback()
  {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = linear_vel * pow(-1, is_reverse); // Additional power is to control the direction
    msg.angular.z = angular_vel;
    if(pub_active)
    {
      cmd_vel_pub->publish(msg);
      if(last_pub)
      {
        pub_active = false;
        last_pub = false;
      }
    }
  }

  //
  // SERVICE CALLBACKS
  //
  void control_callback(const std::shared_ptr<nuturtle_control::srv::Control::Request> request,
                              std::shared_ptr<nuturtle_control::srv::Control::Response> response)
  {
      // Note the received request and update variables accordingly
      linear_vel = (request->velocity) * (request->radius);
      angular_vel = request->radius;

      // Activate publisher
      pub_active = true;

      // Give a successful response
      response->success = true;
  }

  void reverse_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> /* request */,
                              std::shared_ptr<std_srvs::srv::Empty::Response> /* response */)
  {
      is_reverse = !is_reverse;
  }

  void stop_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> /* request */,
                      std::shared_ptr<std_srvs::srv::Empty::Response> /* response */)
  {
      linear_vel = 0.0;
      last_pub = true;
  }

  //
  // HELPER FUNCTIONS
  //
  /// @brief
  void init_var()
  {

  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CircleNode>());
  rclcpp::shutdown();
  return 0;
}