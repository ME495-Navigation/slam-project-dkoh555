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
                          fresh_cmd_vel_received(false),
                          fresh_sensor_data_received(false)
    {
      //
      // PARAMETERS
      //
      auto param_desc = rcl_interfaces::msg::ParameterDescriptor{}; // Prepare for parameter descriptions

      param_desc.description = "Frequency of node timer";
      declare_parameter("frequency", 100.0, param_desc);
      frequency = get_parameter("frequency").as_double();

      param_desc.description = "";
      declare_parameter("wheel_radius", -1.0, param_desc);
      wheel_radius = get_parameter("wheel_radius").as_double();

      param_desc.description = "";
      declare_parameter("track_width", -1.0, param_desc);
      track_width = get_parameter("track_width").as_double();

      param_desc.description = "";
      declare_parameter("motor_cmd_per_rad_sec", -1.0, param_desc);
      motor_cmd_per_rad_sec = get_parameter("motor_cmd_per_rad_sec").as_double();

      param_desc.description = "";
      declare_parameter("encoder_ticks_per_rad", -1.0, param_desc);
      encoder_ticks_per_rad = get_parameter("encoder_ticks_per_rad").as_double();

      if (params_string_unfilled())
      {
          RCLCPP_ERROR(this->get_logger(), "Required paramters not provided");
          rclcpp::shutdown();
      }

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
    double wheel_radius, track_width, motor_cmd_per_rad_sec, encoder_ticks_per_rad;
    bool fresh_cmd_vel_received, fresh_sensor_data_received;

    //
    // Objects
    //
    DiffDrive turtlebot;
    geometry_msgs::msg::Twist received_twist;
    nuturtlebot_msgs::msg::SensorData received_sensordata;
    sensor_msgs::msg::JointState prev_joint_states;
    double curr_sensor_time, prev_sensor_time;

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

        // RCLCPP_INFO(
        //         get_logger(), "wheel left: %f", wheels_delta.left);
        // RCLCPP_INFO(
        //         get_logger(), "wheel right: %f", wheels_delta.right);

        // Use the calculated WheelPosition for forward kinematics to find and update the changes in the robot's configuration
        // (Including wheel angles, and position of the robot)
        // turtlebot.forward_k(wheels_delta);

        // Adjust the found WheelPosition so that it is suitable to be sent to the turtlebot motors
        float raw_left_vel, raw_right_vel;    // Left and right wheel velocity, in "motor command units" (mcu)
                                              // For the turtlebot, each motor can be command with an integer velocity of between
                                              // -265 mcu and 265 mcu, and 1 mcu = 0.024 rad/sec
        raw_left_vel = wheels_delta.left / motor_cmd_per_rad_sec;
        raw_right_vel = wheels_delta.right / motor_cmd_per_rad_sec;

        // RCLCPP_INFO(
        //         get_logger(), "raw left: %f", raw_left_vel);
        // RCLCPP_INFO(
        //         get_logger(), "raw right: %f", raw_right_vel);

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
      // Note the received SensorData message
      received_sensordata = *msg;

      // Placing the joint_states publishing code in the sensor_data_callback so that it publishes
      // the most up-to-date JointStates as soon as any SensorData is received

      // Note the time the SensorData message is received
      curr_sensor_time = received_sensordata.stamp.sec + received_sensordata.stamp.nanosec * 1e-9;

      // Extract the useful data from the received SensorData message
      int left_encoder = received_sensordata.left_encoder;
      int right_encoder = received_sensordata.right_encoder;

      // Convert encoder readings into angle radians wrapped to (-PI, PI]
      double left_angle = ticks_to_rad(left_encoder);
      double right_angle = ticks_to_rad(right_encoder);

      // Fill the JointStates message with the relevant information
      sensor_msgs::msg::JointState robot_joint_states;
      robot_joint_states.header.stamp = received_sensordata.stamp;
      robot_joint_states.name = {"wheel_left_joint", "wheel_right_joint"};

      // Fill in position information
      robot_joint_states.position = {left_angle, right_angle};

      // Fill in velocity information
      // If this is the first SensorData message received, initialize the velocity as zeros
      if(!fresh_sensor_data_received)
      {
        // Velocities set to 0
        robot_joint_states.velocity = {};
        // Previous JointState is set to the current one
        prev_joint_states = robot_joint_states;
        // Previous time of receiving SensorData is set to current one
        prev_sensor_time = curr_sensor_time;
        // Noted that the first SensorData message was received
        fresh_sensor_data_received = true;
      }
      // Else, calculate the velocity using the previous message information
      else
      {
        // Perform necessary velocity calculations
        double time_elapsed = curr_sensor_time - prev_sensor_time;
        double diff_position[2] = {robot_joint_states.position.at(0) - prev_joint_states.position.at(0),
                                  robot_joint_states.position.at(1)- prev_joint_states.position.at(1)};
        // Set the velocities
        robot_joint_states.velocity = {diff_position[0] / time_elapsed, diff_position[1] / time_elapsed};

        // Update the new previous JointState and SensorData time
        prev_joint_states = robot_joint_states;
        prev_sensor_time = curr_sensor_time;
      }

      // Publish the JointState
      joint_states_pub->publish(robot_joint_states);

    }


    //
    // HELPER FUNCTIONS
    //
    /// @brief 
    void init_var()
    {
      // Initialize the diff drive robot
      turtlebot = DiffDrive(wheel_radius, track_width);
    }

    /// @brief Returns encoder ticks converted into angle radians wrapped to (-PI, PI]
    /// @param num_ticks - 
    double ticks_to_rad(int num_ticks)
    {
      double raw_angle = (num_ticks / encoder_ticks_per_rad);
      double normalized_angle = normalize_angle(raw_angle);
      return normalized_angle;
    }

    bool params_string_unfilled()
    {
        return (wheel_radius == -1.0 || track_width == -1.0 || motor_cmd_per_rad_sec == -1.0 || encoder_ticks_per_rad == -1.0);
    }

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleControlNode>());
  rclcpp::shutdown();
  return 0;
}