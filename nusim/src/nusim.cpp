/// @file
/// @brief Controls the simulation of the turtlebot in rviz, including broadcasting transforms
/// 
/// @section Publishers
///   ~/timestep (std_msgs/UInt64) - Current timestep in the simulation
///
/// @section Services
///   ~/reset (std_msgs/Empty) - Service that resets the simulation to its initial state when triggered
///   ~/teleport (nusim/Teleport) - A service that moves the robot to the provided coordinates
///
/// @section Broadcaster
///   ~/walls (MarkerArray) - Marker array that creates the arena walls in rviz
///   ~/obstacles (MarkerArray) - Marker array that creates the various obstacles in rviz
///
/// @section Parameters
///  `~/frequency (double) [default "200.0"]`      - Frequency of node timer
///
///  `~/x (double) [default "0.0"]`      - Starting x coord of robot
///  `~/y (double) [default "0.0"]`      - Starting y coord of robot
///  `~/theta (double) [default "0.0"]`      - Starting directional angle of robot
///  `~/arena_x_length (double) [default "5.0"]`      - x length of arena
///  `~/arena_y_length (double) [default "5.0"]`      - y length of arena
///  `~/obstacles/x (vector<double>) [default "{}"]`      - x coords of each obstacles
///  `~/obstacles/y (vector<double>) [default "{}"]`      - y coords of each obstacles
///  `~/obstacles/r (double) [default "0.75"]`      - Radius of all obstacles

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
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"

#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nusim/srv/teleport.hpp"

#include "turtlelib/diff_drive.hpp"

using std::string;
using namespace turtlelib;

using namespace std::chrono_literals;
using namespace rosnu;
using std::placeholders::_1;
using std::placeholders::_2;

class nusimNode : public rclcpp::Node
{
public:
  //
  // CONSTRUCTOR
  //
  nusimNode() : Node("nusim"), timestep(0), left_vel(0), right_vel(0)
  {
    //
    // PARAMETERS
    //
    // Frequency of timer
    frequency = declare_and_get_param<double>("frequency", 200.0f, *this, "Frequency of node timer");
    x0 = declare_and_get_param<double>("x", 0.0f, *this, "Starting x coord of robot");
    y0 = declare_and_get_param<double>("y", 0.0f, *this, "Starting y coord of robot");
    theta0 = declare_and_get_param<double>("theta", 0.0f, *this, "Starting directional angle of robot");
    arena_x_length = declare_and_get_param<double>("arena_x_length", 5.0f, *this, "x length of arena");
    arena_y_length = declare_and_get_param<double>("arena_y_length", 5.0f, *this, "y length of arena");
    obx_arr = declare_and_get_param<std::vector<double>>("obstacles/x", std::vector<double>{}, *this, "x coords of each obstacles");
    oby_arr = declare_and_get_param<std::vector<double>>("obstacles/y", std::vector<double>{}, *this, "y coords of each obstacles");
    obr = declare_and_get_param<double>("obstacles/r", 0.75f, *this, "Radius of all obstacles");

    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{}; // Prepare for parameter descriptions

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

    // If the x and y obstacle arrays are different size, return an error and exit
    if (obx_arr.size() != oby_arr.size())
    {
      throw std::runtime_error("Mismatching obstacles/x and obstacles/y array sizes");
    }

    //
    // Additional variable initialization
    //
    init_var();

    //
    // SUBSCRIBERS
    //
    wheel_cmd_sub = create_subscription<nuturtlebot_msgs::msg::WheelCommands>("red/wheel_cmd", 10, std::bind(&nusimNode::wheel_cmd_callback, this, _1));

    //
    // PUBLISHERS
    //
    // Publishing the simulator's timestep count
    timestep_pub = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 100);
    // Publishing the wall markers
    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();
    wall_pub = create_publisher<visualization_msgs::msg::MarkerArray>("~/walls", qos);
    obs_pub = create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", qos);
    // Other publishers
    sensor_data_pub = create_publisher<nuturtlebot_msgs::msg::SensorData>("red/sensor_data", 100);

    //
    // SERVICE
    //
    // Service to reset the simulator
    reset_srv = this->create_service<std_srvs::srv::Empty>("~/reset", std::bind(&nusimNode::reset_callback, this, _1, _2));
    // Service to teleport the robot
    teleport_srv = this->create_service<nusim::srv::Teleport>("~/teleport", std::bind(&nusimNode::teleport_callback, this, _1, _2));

    //
    // BROADCASTER
    //
    // Transform
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    //
    // TIMER
    //
    timer_ = this->create_wall_timer(1.0s / frequency, std::bind(&nusimNode::timer_callback, this));
  }

private:
  //
  // Node-related Declarations
  //
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_pub;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_srv;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wall_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obs_pub;
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_sub;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_pub;

  //
  // Variables
  //
  double frequency;
  size_t timestep;
  double x0, y0, theta0; // Starting pos
  double x, y, theta; // Current pos
  double arena_x_length, arena_y_length; // World dimensions
  std::vector<double> obx_arr, oby_arr; // Obstacle coords
  double obr;
  double wheel_radius, track_width, motor_cmd_per_rad_sec, encoder_ticks_per_rad;
  double left_vel, right_vel;

  //
  // Objects
  //
  DiffDrive turtlebot;
  WheelPosition wheel_change;

  //
  // TIMER CALLBACK
  //
  /// @brief Timer callback for node, reads joy_state to publish appropriate output messages
  void timer_callback()
  {
    // Timestep
    std_msgs::msg::UInt64 new_msg;
    new_msg.data = timestep;
    timestep_pub->publish(new_msg);
    timestep += 1;

    // Note change in WheelPosition, and run it through forward_k
    WheelPosition wheel_change{right_vel / frequency, left_vel / frequency};
    turtlebot.forward_k(wheel_change);

    // RCLCPP_INFO(
    //         get_logger(), "left change: %f", left_vel / frequency);
    // RCLCPP_INFO(
    //         get_logger(), "right change: %f", right_vel / frequency);

    // RCLCPP_INFO(
    //         get_logger(), "x: %f", turtlebot.get_position().translation().x);
    // RCLCPP_INFO(
    //         get_logger(), "y: %f", turtlebot.get_position().translation().y);
    
    // Update the current position in nusim
    x = turtlebot.get_position().translation().x;
    y = turtlebot.get_position().translation().y;
    theta = turtlebot.get_position().rotation();

    // Update the sensor data
    nuturtlebot_msgs::msg::SensorData sensor_msg;
    sensor_msg.stamp = rclcpp::Clock().now();

    // Convert these angles to encoder ticks
    sensor_msg.right_encoder = turtlebot.get_wheels().right * encoder_ticks_per_rad;
    sensor_msg.left_encoder = turtlebot.get_wheels().left * encoder_ticks_per_rad;

    // Publish the sensor data
    sensor_data_pub->publish(sensor_msg);

    // Transforms
    tf_world_robot(x, y, theta);

    // World objects
    wall_broadcast();
    obstacle_broadcast();
  }

  //
  // SUBSCRIBER CALLBACKS
  //
  void wheel_cmd_callback(const nuturtlebot_msgs::msg::WheelCommands::SharedPtr msg)
  {
    // Need to convert the MCU into wheel velocities
    // Note the WheelCommand readings
    nuturtlebot_msgs::msg::WheelCommands latest_command = *msg;
    auto left_mcu = latest_command.left_velocity;
    auto right_mcu = latest_command.right_velocity;

    // RCLCPP_INFO(
    //         get_logger(), "left: %d", left_mcu);
    // RCLCPP_INFO(
    //         get_logger(), "right: %d", right_mcu);

    // Convert them into wheel velocities
    left_vel = left_mcu * motor_cmd_per_rad_sec;
    right_vel = right_mcu * motor_cmd_per_rad_sec;
  }

  //
  // SERVICE CALLBACKS
  //
  void reset_callback(std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    init_var();
  }

  void teleport_callback(std::shared_ptr<nusim::srv::Teleport::Request> request, std::shared_ptr<nusim::srv::Teleport::Response> response)
  {
    x = request->x;
    y = request->y;
    theta = request->theta;
    RCLCPP_INFO_STREAM(get_logger(), "Teleporting to [x:" << x << " y:" << y << " theta:" << theta << "]");
    response->success = true;
  }

  //
  // TRANSFORM RELATED
  //
  // Broadcasts world -> red robot transform
  void tf_world_robot(double x_in, double y_in, double theta_in)
  {
    geometry_msgs::msg::TransformStamped msg;

    // Key info
    msg.header.stamp = rclcpp::Clock().now();
    msg.header.frame_id = "nusim/world";
    msg.child_frame_id = "red/base_footprint";

    // Translation
    msg.transform.translation.x = x_in;
    msg.transform.translation.y = y_in;
    msg.transform.translation.z = 0.0;
    
    // Rotation
    tf2::Quaternion quat;
    quat.setRPY(0, 0, theta_in);
    msg.transform.rotation.x = quat.x();
    msg.transform.rotation.y = quat.y();
    msg.transform.rotation.z = quat.z();
    msg.transform.rotation.w = quat.w();

    // Broadcast transform
    tf_broadcaster->sendTransform(msg);
  }

  // Broadcasts world's walls
  void wall_broadcast()
  {
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker wall;

    for (int i = 0; i <= 3; i++) {
      wall.header.stamp = rclcpp::Clock().now();
      wall.header.frame_id = "nusim/world";
      wall.id = i;
      wall.type = 1;
      wall.action = 0;

      wall.color.r = 1.0;
      wall.color.g = 0.0;
      wall.color.b = 0.0;
      wall.color.a = 1.0;

      wall.scale.x = 0.0;
      wall.scale.y = 0.0;
      wall.scale.z = 0.25;
      wall.pose.position.x = 0.0;
      wall.pose.position.y = 0.0;
      wall.pose.position.z = 0.125;
      marker_array.markers.push_back(wall);
    }

    const auto wall_thickness = 0.05;

    marker_array.markers.at(0).scale.x = wall_thickness;
    marker_array.markers.at(0).scale.y = arena_y_length + 2 * wall_thickness;
    marker_array.markers.at(0).pose.position.x = 0.5 * (arena_x_length + wall_thickness);

    marker_array.markers.at(1).scale.x = arena_x_length + 2 * wall_thickness;
    marker_array.markers.at(1).scale.y = wall_thickness;
    marker_array.markers.at(1).pose.position.y = 0.5 * (arena_y_length + wall_thickness);

    marker_array.markers.at(2).scale.x = wall_thickness;
    marker_array.markers.at(2).scale.y = arena_y_length + 2 * wall_thickness;
    marker_array.markers.at(2).pose.position.x = -0.5 * (arena_x_length + wall_thickness);

    marker_array.markers.at(3).scale.x = arena_x_length + 2 * wall_thickness;
    marker_array.markers.at(3).scale.y = wall_thickness;
    marker_array.markers.at(3).pose.position.y = -0.5 * (arena_y_length + wall_thickness);

    wall_pub->publish(marker_array);
  }

  void obstacle_broadcast()
  {
    visualization_msgs::msg::MarkerArray obs_array;
    for (size_t i = 0; i < obx_arr.size(); i++) {
      visualization_msgs::msg::Marker obs;
      obs.header.stamp = rclcpp::Clock().now();
      obs.header.frame_id = "nusim/world";
      obs.id = i;
      obs.type = 3;
      obs.action = 0;

      obs.color.r = 1.0;
      obs.color.g = 0.0;
      obs.color.b = 0.0;
      obs.color.a = 1.0;

      obs.pose.position.x = this->obx_arr[i];
      obs.pose.position.y = this->oby_arr[i];
      obs.pose.position.z = 0.125;
      obs.scale.x = this->obr * 2;
      obs.scale.y = this->obr * 2;
      obs.scale.z = 0.25;
      obs_array.markers.push_back(obs);
    }

    obs_pub->publish(obs_array);
  }

  //
  // HELPER FUNCTIONS
  //
  void init_var()
  { 
    timestep = 0;
    x = x0;
    y = y0;
    theta = theta0;

    // Initialize the turtlebot
    turtlebot = DiffDrive(wheel_radius, track_width, Transform2D(Vector2D{x, y}, theta), WheelPosition()); 
  }

  bool params_string_unfilled()
  {
      return (wheel_radius == -1.0 || track_width == -1.0 || motor_cmd_per_rad_sec == -1.0 || encoder_ticks_per_rad == -1.0);
  }

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nusimNode>());
  rclcpp::shutdown();
  return 0;
}