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
    TurtleControlNode() : Node("turtle_control"),
                          fresh_cmd_vel_received(false)
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
    double fresh_cmd_vel_received;

    //
    // Objects
    //
    DiffDrive turtlebot;
    geometry_msgs::msg::Twist received_twist;

    //
    // TIMER CALLBACK
    //
    /// @brief Timer callback for node, reads joy_state to publish appropriate output messages
    void timer_callback()
    {
      if(fresh_cmd_vel_received)
      {
        // Placing the cmd_vel publishing code in the timer so that it's easier to control
        // its rate of publishing and therefore the rate in which the turtlebot motors receive commands

        // Convert the Twist message to turtlelib's Twist2D message
        Twist2D converted_twist{received_twist.angular.z, received_twist.linear.x, received_twist.linear.y};

        // Use the Twist2D message to for inverse kinematics to calculate how much the wheels have rotated
        WheelPosition wheels_delta = turtlebot.inverse_k(converted_twist);

        // Use the calculated WheelPosition for forward kinematics to find and update the changes in the robot's configuration
        // (Including wheel angles, and position of the robot)
        turtlebot.forward_k(wheels_delta);

        // Adjust the found WheelPosition so that it is suitable to be sent to the turtlebot motors
        float raw_left_vel, raw_right_vel;    // Left and right wheel velocity, in "motor command units" (mcu)
                                              // For the turtlebot, each motor can be command with an integer velocity of between
                                              // -265 mcu and 265 mcu, and 1 mcu = 0.024 rad/sec
        raw_left_vel = wheels_delta.left / 0.024;
        raw_right_vel = wheels_delta.right / 0.024;

        // Convert the raw velocities to integers so that it's compatible with MCU ticks
        int mcu_left_vel = static_cast<int>(std::round(raw_left_vel));
        int mcu_right_vel = static_cast<int>(std::round(raw_right_vel));

        // Use these MCU velocities for a WheelCommands message and publish it
        nuturtlebot_msgs::msg::WheelCommands wheel_command;
        wheel_command.left_velocity = mcu_left_vel;
        wheel_command.right_velocity = mcu_right_vel;
        wheel_cmd_pub->publish(wheel_command);

      }
    }

    //
    // NODE CALLBACKS
    //
    /// @brief 
    /// @param msg - 
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
      // Note the received Twist message
      received_twist = *msg;
      fresh_cmd_vel_received = true;
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