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
#include "geometry_msgs/msg/quaternion.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "turtlelib/diff_drive.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using std::string;

using namespace std::chrono_literals;
using namespace turtlelib;

using std::placeholders::_1;
using std::placeholders::_2;

class OdometryNode : public rclcpp::Node
{
  public:
    //
    // CONSTRUCTOR
    //
    OdometryNode() : Node("odometry")
    {
      //
      // PARAMETERS
      //
      auto param_desc = rcl_interfaces::msg::ParameterDescriptor{}; // Prepare for parameter descriptions

      param_desc.description = "Frequency of node timer";
      declare_parameter("frequency", 95.0, param_desc);
      frequency = get_parameter("frequency").as_double();

      param_desc.description = "Frame ID of body frame";
      declare_parameter("body_id", "UNUSED", param_desc);
      body_id = get_parameter("body_id").as_string();

      param_desc.description = "Frame ID of odom frame";
      declare_parameter("odom_id", "odom", param_desc);
      odom_id = get_parameter("odom_id").as_string();

      param_desc.description = "Frame ID of left wheel";
      declare_parameter("wheel_left", "UNUSED", param_desc);
      wheel_left = get_parameter("wheel_left").as_string();

      param_desc.description = "Frame ID of right wheel";
      declare_parameter("wheel_right", "UNUSED", param_desc);
      wheel_right = get_parameter("wheel_right").as_string();

      if(check_params_string())
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
      joint_states_sub = create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, std::bind(&OdometryNode::joint_states_callback, this, _1));

      //
      // PUBLISHERS
      //
      odom_pub = create_publisher<nav_msgs::msg::Odometry>("odom", 100);

      //
      // SERVICE
      //

      //
      // BROADCASTER
      //

      //
      // TIMER
      //
      timer = create_wall_timer(1.0s / frequency, std::bind(&OdometryNode::timer_callback, this));
    }

  private:
    //
    // Node-related Declarations
    //
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub;

    //
    // Variables
    //
    double frequency;
    std::string body_id, odom_id, wheel_left, wheel_right; 

    //
    // Objects
    //
    DiffDrive turtlebot;
    sensor_msgs::msg::JointState latest_joint_states;

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
    void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Note the received JointState message
        latest_joint_states = *msg;

        // Note the new WheelPosition
        WheelPosition curr_wheel_position{latest_joint_states.position[1], latest_joint_states.position[0]};
        WheelPosition old_wheel_position = turtlebot.get_wheels();

        // Note the change in WheelPosition
        WheelPosition delta_wheel_position{curr_wheel_position.right - old_wheel_position.right,
                                            curr_wheel_position.left - old_wheel_position.left};
        
        // Update the robot configuration with forward_k
        turtlebot.forward_k(delta_wheel_position);

        // Publish the corresponding odometry message
        // Initialize the odometry message
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = latest_joint_states.header.stamp;
        odom_msg.header.frame_id = odom_id;

        // Fill in the positional information
        geometry_msgs::msg::Pose robot_pose;
        robot_pose.position.x = turtlebot.get_position().translation().x;
        robot_pose.position.y = turtlebot.get_position().translation().y;

        // Use a tf2 object and function to convert roll pitch yaw into quaternion
        tf2::Quaternion tf2_robot_quaternion;
        tf2_robot_quaternion.setRPY(0.0, 0.0, turtlebot.get_position().rotation());
        tf2_robot_quaternion.normalize(); // Ensure the quaternion's magnitude is 1
        // Then convert that tf2 quaternion into a suitable quaternion message
        geometry_msgs::msg::Quaternion robot_quaternion;
        tf2::convert(tf2_robot_quaternion, robot_quaternion);
        robot_pose.orientation = robot_quaternion;
        
        
    }

    //
    // HELPER FUNCTIONS
    //
    /// @brief 
    void init_var()
    {
      // Initialize the diff drive robot
      turtlebot = DiffDrive();
    }

    bool check_params_string()
    {
        return (body_id == "UNUSED" || wheel_left == "UNUSED" || wheel_right == "UNUSED");
    }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryNode>());
  rclcpp::shutdown();
  return 0;
}