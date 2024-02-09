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
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "nusim/srv/teleport.hpp"

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

        param_desc.description = "";
        declare_parameter("wheel_radius", -1.0, param_desc);
        wheel_radius = get_parameter("wheel_radius").as_double();

        param_desc.description = "";
        declare_parameter("track_width", -1.0, param_desc);
        track_width = get_parameter("track_width").as_double();

        param_desc.description = "";
        declare_parameter("motor_cmd_max", -1.0, param_desc);
        motor_cmd_max = get_parameter("motor_cmd_max").as_double();

        param_desc.description = "";
        declare_parameter("motor_cmd_per_rad_sec", -1.0, param_desc);
        motor_cmd_per_rad_sec = get_parameter("motor_cmd_per_rad_sec").as_double();

        param_desc.description = "";
        declare_parameter("encoder_ticks_per_rad", -1.0, param_desc);
        encoder_ticks_per_rad = get_parameter("encoder_ticks_per_rad").as_double();

        if (params_double_unfilled())
        {
            RCLCPP_ERROR(this->get_logger(), "Required paramters not provided");
            rclcpp::shutdown();
        }

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
        joint_states_sub = create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, std::bind(&OdometryNode::joint_states_callback, this, _1));

        //
        // PUBLISHERS
        //
        odom_pub = create_publisher<nav_msgs::msg::Odometry>("odom", 100);

        //
        // SERVICE
        //
        initial_pose_srv = create_service<nusim::srv::Teleport>("initial_pose", std::bind(&OdometryNode::initial_pose_callback, this, _1, _2));

        //
        // BROADCASTER
        //
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

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
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub;
    rclcpp::Service<nusim::srv::Teleport>::SharedPtr initial_pose_srv;

    //
    // Variables
    //
    double frequency;
    double wheel_radius, track_width, motor_cmd_max, motor_cmd_per_rad_sec, encoder_ticks_per_rad;
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
    // SUBSCRIBER CALLBACKS
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
        odom_msg.child_frame_id = body_id;

        // Fill in the positional information
        geometry_msgs::msg::Pose robot_pose;
        robot_pose.position.x = turtlebot.get_position().translation().x;
        robot_pose.position.y = turtlebot.get_position().translation().y;

        // Use a tf2 object and function to convert roll pitch yaw into quaternion
        tf2::Quaternion tf2_robot_quaternion;
        tf2_robot_quaternion.setRPY(0.0, 0.0, turtlebot.get_position().rotation());
        // tf2_robot_quaternion.normalize(); // Ensure the quaternion's magnitude is 1
        // Then convert that tf2 quaternion into a suitable quaternion message

        // geometry_msgs::msg::Quaternion robot_quaternion;
        // tf2::convert(tf2_robot_quaternion, robot_quaternion);
        robot_pose.orientation.x = tf2_robot_quaternion.x();
        robot_pose.orientation.y = tf2_robot_quaternion.y();
        robot_pose.orientation.z = tf2_robot_quaternion.z();
        robot_pose.orientation.w = tf2_robot_quaternion.w();

        // Retrieve the current twist of the robot and fill in twist message information
        Twist2D curr_twist = turtlebot.get_twist();
        geometry_msgs::msg::Twist robot_twist;
        robot_twist.linear.x = curr_twist.x;
        robot_twist.linear.y = curr_twist.y;
        robot_twist.angular.z = curr_twist.omega;

        // Combine different components into the odometry message
        odom_msg.pose.pose = robot_pose;
        odom_msg.twist.twist = robot_twist;
        odom_pub->publish(odom_msg);

        // Broadcast the TF from odom to body
        tf_odom_robot(turtlebot.get_position().translation().x,
                    turtlebot.get_position().translation().y, turtlebot.get_position().rotation());
    }

    //
    // SERVICE CALLBACKS
    //
    void initial_pose_callback(const std::shared_ptr<nusim::srv::Teleport::Request> request,
                                std::shared_ptr<nusim::srv::Teleport::Response> response)
    {
        // Note the received request as a Transform2D
        Transform2D received_trans{Vector2D{request->x, request->y}, request->theta};
        // Set the position configuration of the robot
        turtlebot.set_transform(received_trans);

        // Broadcast the TF from odom to body
        tf_odom_robot(turtlebot.get_position().translation().x,
                    turtlebot.get_position().translation().y, turtlebot.get_position().rotation());

        // Give a successful response
        response->success = true;
    }

    //
    // TRANSFORM RELATED
    //
    // Broadcasts world -> red robot transform
    void tf_odom_robot(double x_in, double y_in, double theta_in)
    {
        geometry_msgs::msg::TransformStamped msg;

        // Key info
        msg.header.stamp = latest_joint_states.header.stamp;
        msg.header.frame_id = odom_id;
        msg.child_frame_id = body_id;

        // Translation
        msg.transform.translation.x = x_in;
        msg.transform.translation.y = y_in;
        msg.transform.translation.z = 0.0;

        // Rotation
        tf2::Quaternion raw_quat;
        raw_quat.setRPY(0, 0, theta_in);
        // raw_quat.normalize();

        // geometry_msgs::msg::Quaternion quat;
        // tf2::convert(raw_quat, quat);
        msg.transform.rotation.x = raw_quat.x();
        msg.transform.rotation.y = raw_quat.y();
        msg.transform.rotation.z = raw_quat.z();
        msg.transform.rotation.w = raw_quat.w();


        // Broadcast transform
        tf_broadcaster->sendTransform(msg);
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

    bool params_string_unfilled()
    {
        return (body_id == "UNUSED" || wheel_left == "UNUSED" || wheel_right == "UNUSED");
    }


    bool params_double_unfilled()
    {
        return (wheel_radius == -1.0 || motor_cmd_max == -1.0 || track_width == -1.0 || motor_cmd_per_rad_sec == -1.0 || encoder_ticks_per_rad == -1.0);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryNode>());
    rclcpp::shutdown();
    return 0;
}