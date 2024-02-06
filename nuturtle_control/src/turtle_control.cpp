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

using std::string;

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class TurtleControlNode : public rclcpp::Node
{
  public:
    //
    // CONSTRUCTOR
    //
    TurtleControlNode() : Node("turtle_control")
    {
      //
      // PARAMETERS
      //
      auto param_desc = rcl_interfaces::msg::ParameterDescriptor{}; // Prepare for parameter descriptions

      param_desc.description = "Frequency of node timer";
      declare_parameter("frequency", 95.0, param_desc);
      frequency = get_parameter("frequency").as_double();

      //
      // Additional variable initialization
      //
      init_var();

      //
      // PUBLISHERS
      //
      cmd_vel_pub = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 100);

      //
      // SERVICE
      //

      //
      // BROADCASTER
      //

      //
      // TIMER
      //
      timer = create_wall_timer(1.0s / frequency, std::bind(&TurtleControlNode::timer_callback, this));
    }

  private:
    //
    // Node-related Declarations
    //
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;

    //
    // Variables
    //
    double frequency;

    //
    // TIMER CALLBACK
    //
    /// @brief Timer callback for node, reads joy_state to publish appropriate output messages
    void timer_callback()
    {

    }

    //
    // HELPER FUNCTIONS
    //
    void init_var()
    {

    }

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleControlNode>());
  rclcpp::shutdown();
  return 0;
}