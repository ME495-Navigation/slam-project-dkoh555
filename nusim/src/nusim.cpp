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
#include "nusim/param_lib.hpp"
#include "nusim/random_num_generator.hpp"
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
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"

#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nusim/srv/teleport.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "turtlelib/diff_drive.hpp"

using std::string;
using namespace turtlelib;

using namespace std::chrono_literals;
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
    frequency = rosnu::declare_and_get_param<double>("frequency", 200.0f, *this, "Frequency of node timer");
    x0 = rosnu::declare_and_get_param<double>("x", 0.0f, *this, "Starting x coord of robot");
    y0 = rosnu::declare_and_get_param<double>("y", 0.0f, *this, "Starting y coord of robot");
    theta0 = rosnu::declare_and_get_param<double>("theta", 0.0f, *this, "Starting directional angle of robot");
    arena_x_length = rosnu::declare_and_get_param<double>("arena_x_length", 5.0f, *this, "x length of arena");
    arena_y_length = rosnu::declare_and_get_param<double>("arena_y_length", 5.0f, *this, "y length of arena");
    obx_arr = rosnu::declare_and_get_param<std::vector<double>>("obstacles/x", std::vector<double>{}, *this, "x coords of each obstacles");
    oby_arr = rosnu::declare_and_get_param<std::vector<double>>("obstacles/y", std::vector<double>{}, *this, "y coords of each obstacles");
    obr = rosnu::declare_and_get_param<double>("obstacles/r", 0.75f, *this, "Radius of all obstacles");
    input_noise = rosnu::declare_and_get_param<double>("input_noise", 0.2f, *this, "The amount of noise the sensor receives");
    slip_fraction = rosnu::declare_and_get_param<double>("slip_fraction", 0.2f, *this, "The amount of slipping that the wheels encounter");
    // Basic Sensor Parameters
    basic_sensor_variance = rosnu::declare_and_get_param<double>("basic_sensor_variance", 0.01f, *this, "The amount of noise for sensor data");
    max_range = rosnu::declare_and_get_param<double>("max_range", 1.0f, *this, "The sensor range for the simulated robot");
    // Lidar Parameters
    min_lidar_range = rosnu::declare_and_get_param<double>("min_lidar_range", 0.1f, *this, "");
    max_lidar_range = rosnu::declare_and_get_param<double>("max_lidar_range", 1.0f, *this, "");
    lidar_angle_incr = rosnu::declare_and_get_param<double>("lidar_angle_incr", 0.05f, *this, "");
    lidar_resolution = rosnu::declare_and_get_param<double>("lidar_resolution", 0.0001f, *this, "");
    lidar_noise_level = rosnu::declare_and_get_param<double>("lidar_noise_level", 0.1f, *this, "");


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

    param_desc.description = "";
    declare_parameter("collision_radius", -1.0, param_desc);
    collision_radius = get_parameter("collision_radius").as_double();

    if (params_unfilled())
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
    // Publishing markers
    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();
    wall_pub = create_publisher<visualization_msgs::msg::MarkerArray>("~/walls", qos);
    obs_pub = create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", qos);
    fake_sensor_pub = create_publisher<visualization_msgs::msg::MarkerArray>("~/fake_sensor", qos);
    // Publishing sensor data
    sensor_data_pub = create_publisher<nuturtlebot_msgs::msg::SensorData>("red/sensor_data", 100);
    // Publishing fake lidar data
    fake_lidar_pub = create_publisher<sensor_msgs::msg::LaserScan>("~/fake_lidar", 100);
    // Publishing the path taken
    path_pub = create_publisher<nav_msgs::msg::Path>("nav_msgs/path", 100);

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
    tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

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
  std::unique_ptr<tf2_ros::Buffer> tfBuffer;
  std::shared_ptr<tf2_ros::TransformListener> tfListener;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_pub;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_srv;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wall_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obs_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_pub;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr fake_lidar_pub;
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_sub;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;

  //
  // Variables
  //
  double frequency;
  size_t timestep;
  double x0, y0, theta0; // Starting pos
  double x, y, theta; // Current pos
  double x_scan, y_scan; // Translation on ground plane from base_footprint to base_scan
  double arena_x_length, arena_y_length; // World dimensions
  std::vector<double> obx_arr, oby_arr; // Obstacle coords
  double obr;
  double wheel_radius, track_width, motor_cmd_per_rad_sec, encoder_ticks_per_rad, collision_radius;
  double left_vel, right_vel;
  double input_noise;
  double slip_fraction;
  rclcpp::Time start_period;
  double basic_sensor_variance;
  double max_range;
  double min_lidar_range, max_lidar_range, lidar_angle_incr, lidar_resolution, lidar_noise_level;

  //
  // Objects
  //
  DiffDrive turtlebot;
  WheelPosition wheel_change;
  std::normal_distribution<> input_noise_generator;
  std::uniform_real_distribution<> slip_noise_generator;
  std::normal_distribution<> obstacle_noise_generator;

  // Vectors
  std::vector<geometry_msgs::msg::PoseStamped> robot_path;

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

    // Calculate the new wheel positions due to velocity,
    // also apply uniform random noise to each wheel velocity
    auto right_pos = (right_vel * (1 + slip_noise_generator(num_generator::get_random()))) / frequency;
    auto left_pos = (left_vel * (1 + slip_noise_generator(num_generator::get_random()))) / frequency;
    // Note changes in WheelPosition, and run it through forward_k
    WheelPosition wheel_change{right_pos, left_pos};
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

    // Check for collisions between robot and obstacles and correct robot position accordingly
    collision_update();

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

    // Navigtaion Path
    nav_path_publish();

    // World objects
    wall_broadcast();
    obstacle_broadcast();

    // Need to check if a time period of 1/5 HZ passed to then
    // publish fake sensor data
    rclcpp::Time end_period = rclcpp::Clock().now();
    if((end_period - start_period) >= rclcpp::Duration(0,2e08))
    {
      fake_sensor_broadcast();
      fake_lidar_broadcast();
      start_period = end_period;
    }
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

    // Add noise in form of a gaussian random variable to the wheel velocities for simulation purposes,
    // but if the velocity is 0 we'll leave it untouched because we're confident in that
    if(left_vel != 0.0)
    {
      left_vel += input_noise_generator(num_generator::get_random());
    }
    if(right_vel != 0.0)
    {
      right_vel += input_noise_generator(num_generator::get_random());
    }
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
    // RCLCPP_INFO_STREAM(get_logger(), "Teleporting to [x:" << x << " y:" << y << " theta:" << theta << "]");
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

    // Apply the same transformations to a PoseStamped msg
    // and then append it to the robot_path vector
    geometry_msgs::msg::PoseStamped pose_stamp;

    // Key info
    pose_stamp.header = msg.header;
    
    // Position
    pose_stamp.pose.position.x = msg.transform.translation.x;
    pose_stamp.pose.position.y = msg.transform.translation.y;
    pose_stamp.pose.position.z = msg.transform.translation.z;

    // Orientation
    pose_stamp.pose.orientation.x = msg.transform.rotation.x;
    pose_stamp.pose.orientation.y = msg.transform.rotation.y;
    pose_stamp.pose.orientation.z = msg.transform.rotation.z;
    pose_stamp.pose.orientation.w = msg.transform.rotation.w;

    // Append it to robot_path vector
    robot_path.push_back(pose_stamp);

    // Broadcast transform
    tf_broadcaster->sendTransform(msg);
  }

  // Publish the robot path trace
  void nav_path_publish()
  {
    nav_msgs::msg::Path msg;

    // Key info
    msg.header.stamp = rclcpp::Clock().now();
    msg.header.frame_id = "nusim/world";

    // Vector of Poses
    msg.poses = robot_path;

    // Publish the path trace
    path_pub->publish(msg);
  }

  void check_base_scan_tf()
  {
    // Finding the position of the base_scan frame relative to the base_footprint frame
    try
    {
      const auto tf_footprint_scan = tfBuffer->lookupTransform("red/base_footprint", "red/base_scan", tf2::TimePointZero);

      // Note the vector
      x_scan = tf_footprint_scan.transform.translation.x;
      y_scan = tf_footprint_scan.transform.translation.y;
    }
    catch (const tf2::TransformException & ex) {
      RCLCPP_DEBUG(get_logger(),  ex.what());
    }
  }

  //
  // PUBLISHER FUNCTIONS
  //
  void fake_lidar_broadcast()
  {
    // Fill in the LaserScan message fields
    sensor_msgs::msg::LaserScan msg;
    msg.ranges.clear(); // Clear the ranges array to get rid of "ghost" points
    msg.header.stamp = rclcpp::Clock().now();
    msg.header.frame_id = "red/base_scan";
    msg.angle_min = 0.0;
    msg.angle_max = 2 * PI;
    msg.angle_increment = lidar_angle_incr;
    msg.time_increment = 0.0;
    msg.scan_time = 1/5;
    msg.range_min = min_lidar_range;
    msg.range_max = max_lidar_range;

    // For accurate real-life representation of lidar readings, need the tf of world to turtlebot scanner.
    //
    if(x_scan == 0.0 && y_scan == 0.0)
    {
      check_base_scan_tf();
    }
    // tf of robot footprint to scanner
    const Transform2D tf_footprint_scanner(Vector2D{x_scan, y_scan});
    // tf of world to robot scanner (on the ground plane)
    const Transform2D tf_world_scanner = turtlebot.get_position() * tf_footprint_scanner;
    // FROM NOW ON, ANY MENTION OF ROBOT MEANS THE ROBOT SCANNER IN THIS FUNCTION

    // Find the total number of measurements involved in one scan (dependent on angle increment parameter)
    const auto num_measurements_raw = 360.0/lidar_angle_incr;
    const auto num_measurement = static_cast<int>(num_measurements_raw);

    // Initialize the vector of ranges for the lidar message
    std::vector<float> range_arr;

    // Iterate through every measurement
    for (int i = 0; i < num_measurement; i++)
    {
      // Initialize the lidar range value for that measurement
      float lidar_range = 0.0;

      // For this measurement and its angle,
      // Find the min and max range points tfs relative to the robot frame
      // tf of robot to direction of min range point
      const Transform2D tf_r_min_theta(i * lidar_angle_incr);
      // tf of direction of min range point to the min range point
      const Transform2D tf_min_theta_min(Vector2D{min_lidar_range, 0.0});
      // tf of robot to min range point
      const Transform2D tf_r_min = tf_r_min_theta * tf_min_theta_min;
      // tf of min range point to max range point
      const Transform2D tf_min_max(Vector2D{max_lidar_range - min_lidar_range, 0.0});
      // tf of robot to max range point
      const Transform2D tf_r_max = tf_r_min * tf_min_max;

      // Iterate through each obstacle in the world
      for (size_t i = 0; i < obx_arr.size(); i++)
      {
        const auto obx = this->obx_arr[i];
        const auto oby = this->oby_arr[i];
        // Find the min and max range point tfs relative to the obstacle,
        // tf of object to world
        const Transform2D tf_ob_world(Vector2D{-obx, -oby});
        // tf of obstacle to min range point (object to world -> world to robot -> robot to min range point)
        const Transform2D tf_ob_min = tf_ob_world * tf_world_scanner * tf_r_min;
        // tf of obstacle to max range point (object to world -> world to robot -> robot to max range point)
        const Transform2D tf_ob_max = tf_ob_world * tf_world_scanner * tf_r_max;

        // Use the Circle-Line method to determine if the measurement captures the circle
        const auto min_x = tf_ob_min.translation().x;
        const auto min_y = tf_ob_min.translation().y;
        const auto max_x = tf_ob_max.translation().x;
        const auto max_y = tf_ob_max.translation().y;
        // Vector from obstacle frame to intersection point
        const Vector2D nearest_intersect = nearest_circle_line_intersection(min_x, min_y, max_x, max_y, obr);

        // If the Vector calculated contains NAN values, then there is no intersect
        // therefore iterate the for loop to the next obstacle
        if(std::isnan(nearest_intersect.x) || std::isnan(nearest_intersect.y))
        {
          continue;
        }

        // To deal with 'duplicate intersection points',
        // check the sign of the vector relative to the robot between the min range point and intersection point.
        // tf of obstacle to intersection point
        const Transform2D tf_ob_point = Transform2D{nearest_intersect};
        // tf of robot to intersection point
        const Transform2D tf_r_point = tf_world_scanner.inv() * tf_ob_world.inv() * tf_ob_point;
        // Compare the signs of both tf vectors
        const auto sgn_min_x = std::pow(-1, std::signbit(tf_r_min.translation().x));
        const auto sgn_min_y = std::pow(-1, std::signbit(tf_r_min.translation().y));
        const auto sgn_point_x = std::pow(-1, std::signbit(tf_r_point.translation().x));
        const auto sgn_point_y = std::pow(-1, std::signbit(tf_r_point.translation().y));
        // If the signs are opposite then the measurement is a false reading,
        // therefore iterate the for loop to the next obstacle
        if((sgn_min_x == -1 * sgn_point_x) && (sgn_min_y == -1 * sgn_point_y))
        {
          continue;
        }

        // Find the distance between the measurement and the min range point
        const auto dist_intersect_min = euclidian_distance(min_x, min_y, nearest_intersect.x, nearest_intersect.y);
        // Find the raw distance measurement between the intersect point of the obstacle and the robot
        const auto distance_measurement_raw = min_lidar_range + dist_intersect_min;

        // Convert this raw distance measurement according to the resolution parameter
        const auto resolution_count_raw = distance_measurement_raw / lidar_resolution;
        const auto resolution_count_int = static_cast<int>(resolution_count_raw);
        const double adjusted_distance_measurement = resolution_count_int * lidar_resolution;

        // Assign the adjusted measurement to lidar_range
        lidar_range = adjusted_distance_measurement;
        // Once lidar_range is assigned, no other object should be detected so break loop
        break;
      }

      // Add lidar_range to range_array
      msg.ranges.push_back(lidar_range);

      // Publish the message
      fake_lidar_pub->publish(msg);
    }
    
  }

  //
  // MARKER FUNCTIONS
  //

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

  void fake_sensor_broadcast()
  {
    visualization_msgs::msg::MarkerArray obs_array;
    for (size_t i = 0; i < obx_arr.size(); i++) {
      visualization_msgs::msg::Marker obs;
      obs.header.stamp = rclcpp::Clock().now();
      obs.header.frame_id = "nusim/world";
      obs.id = i;
      obs.type = 3;
      // Check if the object is within the sensor range, if so add it, if not delete it
      if(in_range(x, y, this->obx_arr[i], this->oby_arr[i], max_range))
      {
        obs.action = 0;
      }
      else
      {
        obs.action = 2;
      }

      obs.color.r = 0.0;
      obs.color.g = 0.0;
      obs.color.b = 1.0;
      obs.color.a = 1.0;

      // Add in the Gaussian noise
      obs.pose.position.x = this->obx_arr[i] + obstacle_noise_generator(num_generator::get_random());
      obs.pose.position.y = this->oby_arr[i] + obstacle_noise_generator(num_generator::get_random());
      obs.pose.position.z = 0.125;
      obs.scale.x = this->obr * 2;
      obs.scale.y = this->obr * 2;
      obs.scale.z = 0.25;
      obs_array.markers.push_back(obs);
    }

    fake_sensor_pub->publish(obs_array);
  }

  void collision_update()
  {
    // Iterate through each obstacle, and check if robot has collided with it
    for (size_t i = 0; i < obx_arr.size(); i++) {
      // If robot and obstacle collide, then adjust its position so that both
      // objects' collision circles are tangent to each other
      if(in_range(x, y, this->obx_arr[i], this->oby_arr[i], collision_radius + obr))
      {
        // RCLCPP_INFO_STREAM(
        //         get_logger(), "Colliding");

        // RCLCPP_INFO_STREAM(
        //         get_logger(), "collision rad: " << collision_radius << " obr: " << obr);

        // RCLCPP_INFO_STREAM(
        //         get_logger(), "overall_collision_rad: " << collision_radius + obr);

        // RCLCPP_INFO_STREAM(
        //         get_logger(), "x: " << x << " y: " << y);

        // RCLCPP_INFO_STREAM(
        //         get_logger(), "obj_x: " << this->obx_arr[i] << " obj_y: " << this->oby_arr[i]);

        // RCLCPP_INFO_STREAM(
        //         get_logger(), "in_range_rad_sqrd: " << std::pow(collision_radius + obr, 2));
        
        // RCLCPP_INFO_STREAM(
        //         get_logger(), "distance_sqrd: "<< (std::pow(x - this->obx_arr[i], 2) + std::pow(y - this->oby_arr[i], 2)));

        // Find location of tangent_point between robot and obstacle collision circles along the 'path between both objects'.
        // Find the vector from the obstacle frame to the robot's current position
        auto curr_diff_x = x - this->obx_arr[i];
        auto curr_diff_y = y - this->oby_arr[i];
        // Convert it to a unit vector
        auto old_mag = std::sqrt(std::pow(curr_diff_x, 2) + std::pow(curr_diff_y, 2));
        auto unit_diff_x = curr_diff_x / old_mag;
        auto unit_diff_y = curr_diff_y / old_mag;
        // Adjust the magnitude of said vector so that the robot
        // is the correct distance away from the obstacle center
        auto new_mag_x = unit_diff_x * (collision_radius + obr);
        auto new_mag_y = unit_diff_y * (collision_radius + obr);
        // Adjust these tf coordinates of robot from obstacle from the obstacle frame to the world frame
        x = this->obx_arr[i] + new_mag_x;
        y = this->oby_arr[i] + new_mag_y;
        // Ensure that these positions are also noted in nusim's turtlerobot object
        Transform2D new_transform(Vector2D{x, y}, turtlebot.get_position().rotation());
        turtlebot.set_transform(new_transform);
      }
    }
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

    x_scan = 0.0;
    y_scan = 0.0;

    // Initialize the turtlebot
    turtlebot = DiffDrive(wheel_radius, track_width, Transform2D(Vector2D{x, y}, theta), WheelPosition()); 

    // Generate a noise gaussian variable
    input_noise_generator = std::normal_distribution<>{0, input_noise};

    // Generate a slip uniform random variable
    slip_noise_generator = std::uniform_real_distribution<>{-slip_fraction, slip_fraction};

    // Generate a noise gaussian variable
    obstacle_noise_generator = std::normal_distribution<>{0, basic_sensor_variance};

    // Initialize a clock reading for tracking fake sensor data
    start_period = rclcpp::Clock().now();

  }

  bool params_unfilled()
  {
      return (wheel_radius == -1.0 || track_width == -1.0 || motor_cmd_per_rad_sec == -1.0 ||
        encoder_ticks_per_rad == -1.0 || collision_radius == -1.0);
  }

  bool in_range(double x1, double y1, double x2, double y2, double radius)
  {
    double distance_sqrd = std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2);
    return distance_sqrd < std::pow(radius, 2);
  }

  Vector2D nearest_circle_line_intersection(double min_x, double min_y, double max_x, double max_y, double radius)
  {
    // Math referenced from: https://mathworld.wolfram.com/Circle-LineIntersection.html
    // Define important variables
    const auto dx = max_x - min_x;
    const auto dy = max_y - min_y;
    const auto dr = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
    const auto D = min_x * max_y - max_x * min_y;

    // Calculate discriminant
    const auto discriminant = (std::pow(radius, 2) * std::pow(dr, 2)) - std::pow(D, 2);
    // If the discriminant is less than 0, then no intersection, and return a NAN vector
    if(discriminant < 0.0)
    {
      return Vector2D{NAN, NAN};
    }

    // Additional variables
    const auto sgn_dy = std::pow(-1, std::signbit(dy));
    const auto mag_dy = std::abs(dy);

    // Find the points of intersection
    const auto x_1 = ((D * dy) + (sgn_dy * dx * std::sqrt(std::pow(radius, 2) * std::pow(dr, 2) - std::pow(D, 2))))/(std::pow(dr, 2));
    const auto y_1 = (((-D * dx) + (mag_dy * std::sqrt(std::pow(radius, 2) * std::pow(dr, 2) - std::pow(D, 2)))))/(std::pow(dr, 2));

    const auto x_2 = ((D * dy) - (sgn_dy * dx * std::sqrt(std::pow(radius, 2) * std::pow(dr, 2) - std::pow(D, 2))))/(std::pow(dr, 2));
    const auto y_2 = (((-D * dx) - (mag_dy * std::sqrt(std::pow(radius, 2) * std::pow(dr, 2) - std::pow(D, 2)))))/(std::pow(dr, 2));

    // If discriminant is more than 0, then two intersection points, and return the closest point to min range point
    if(discriminant > 0)
    {
      const auto dist_1 = euclidian_distance(min_x, min_y, x_1, y_1);
      const auto dist_2 = euclidian_distance(min_x, min_y, x_2, y_2);

      // If intersect point 1 is closer, return that
      if(dist_1 < dist_2)
      {
        return Vector2D{x_1, y_1};
      }
      // Else, return point 2
      else
      {
        return Vector2D{x_2, y_2};
      }
    }
    // Else, the discriminant would be 0, and the two points would be the same (AKA the tangent), thus return either point
    else
    {
      return Vector2D{x_1, y_1};
    }
  }

  double euclidian_distance(double x1, double y1, double x2, double y2)
  {
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
  }

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nusimNode>());
  rclcpp::shutdown();
  return 0;
}