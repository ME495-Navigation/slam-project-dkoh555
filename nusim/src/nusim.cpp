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

#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nusim/srv/teleport.hpp"

using std::string;

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
  nusimNode() : Node("nusim"), timestep(0)
  {
    //
    // PARAMETERS
    //
    // Frequency of timer
    const auto frequency = declare_and_get_param<double>("frequency", 200.0f, *this, "Frequency of node timer");
    x0 = declare_and_get_param<double>("x", 0.0f, *this, "Starting x coord of robot");
    y0 = declare_and_get_param<double>("y", 0.0f, *this, "Starting y coord of robot");
    theta0 = declare_and_get_param<double>("theta", 0.0f, *this, "Starting directional angle of robot");

    //
    // Additional variable initialization
    //
    init_var();

    //
    // PUBLISHERS
    //
    // Publishing the simulator's timestep count
    timestep_pub = this->create_publisher<std_msgs::msg::UInt64>("timestep", 100);

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

  //
  // Variables
  //
  size_t timestep;
  double x0, y0, theta0; // Starting pos
  double x, y, theta; // Current pos

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
    RCLCPP_INFO_STREAM(get_logger(), "Curr pos [x:" << x << " y:" << y << " theta:" << theta << "]");

    // Transforms
    tf_world_robot(x, y, theta);
  }

  //
  // SERVICE CALLBACKS
  //
  void reset_callback(std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    // timestep = 0;
    // x = x0;
    // y = y0;
    // theta = theta0;
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

  //
  // HELPER FUNCTIONS
  //
  void init_var()
  {
    timestep = 0;
    x = x0;
    y = y0;
    theta = theta0;
  }

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nusimNode>());
  rclcpp::shutdown();
  return 0;
}