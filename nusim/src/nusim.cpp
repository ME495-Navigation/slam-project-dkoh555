#include "rclcpp/rclcpp.hpp"
#include "param_lib.cpp"
#include <string>
#include <cmath>
#include "yaml-cpp/yaml.h"
#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <map>

using std::string;

using namespace std::chrono_literals;
using namespace rosnu;
using std::placeholders::_1;

class nusimNode : public rclcpp::Node
{
public:
  //
  // CONSTRUCTOR
  //
  nusimNode() : Node("nusim")
  {
    //
    // PARAMETERS
    //
    // Frequency of timer
    auto frequency = declare_and_get_param<double>("frequency", 200.0f, *this, "Frequency node timer");

    //
    // TIMER
    //
    timer_ = this->create_wall_timer(1.0s / frequency, std::bind(&nusimNode::timer_callback, this));
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;

  //
  // TIMER CALLBACK
  //
  /// @brief Timer callback for node, reads joy_state to publish appropriate output messages
  void timer_callback()
  {
    RCLCPP_INFO(rclcpp::get_logger("nusim"), "TESTING");
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nusimNode>());
  rclcpp::shutdown();
  return 0;
}