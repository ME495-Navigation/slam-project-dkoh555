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

#include "turtlelib/diff_drive.hpp"

using std::string;

using namespace std::chrono_literals;
using namespace turtlelib;

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
      // SUBSCRIBERS
      //
      cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&TurtleControlNode::cmd_vel_callback, this, _1));
      sensor_data_sub = create_subscription<nuturtlebot_msgs::msg::SensorData>("sensor_data", 10, std::bind(&TurtleControlNode::sensor_data_callback, this, _1));


      //
      // PUBLISHERS
      //
      wheel_cmd_pub = create_publisher<nuturtlebot_msgs::msg::WheelCommands>("wheel_cmd", 100);
      joint_states_pub = create_publisher<sensor_msgs::msg::JointState>("joint_states", 100);

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
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
    rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_sub;
    rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_pub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub;

    //
    // Variables
    //
    double frequency;

    //
    // Variables
    //
    DiffDrive turtlebot;

    //
    // TIMER CALLBACK
    //
    /// @brief Timer callback for node, reads joy_state to publish appropriate output messages
    void timer_callback()
    {

    }

    //
    // NODE CALLBACKS
    //
    /// @brief 
    /// @param msg - 
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
      
    }

    /// @brief 
    /// @param msg - 
    void sensor_data_callback(const nuturtlebot_msgs::msg::SensorData::SharedPtr msg)
    {

    }


    //
    // HELPER FUNCTIONS
    //
    void init_var()
    {
      // Initialize the diff drive robot
      turtlebot = DiffDrive();
    }

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleControlNode>());
  rclcpp::shutdown();
  return 0;
}