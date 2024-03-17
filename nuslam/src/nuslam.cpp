/// @file
/// @brief Implements SLAM for a differential drive robot using sensor data to update and correct robot's position.
///
/// @details This node integrates wheel odometry and sensor data (simulated or real) to maintain an updated map of
///          the environment and the robot's position within it. Utilizes an Extended Kalman Filter (EKF) for SLAM.
///
/// @section publishers Publishers
///   - /odom (nav_msgs/Odometry): Publishes the estimated odometry of the robot from SLAM.
///   - /nav_msgs/path (nav_msgs/Path): Publishes the path taken by the robot as inferred by SLAM.
///   - ~/obstacles (visualization_msgs/MarkerArray): Publishes markers representing the estimated positions of obstacles.
///
/// @section subscribers Subscribers
///   - /joint_states (sensor_msgs/JointState): Subscribes to the joint states of the robot to estimate motion.
///   - /fake_sensor (visualization_msgs/MarkerArray): Subscribes to simulated sensor data for landmarks detection.
///
/// @section services Services
///   - /initial_pose (nusim/Teleport): Service to set the initial pose of the robot in the map.
///
/// @section broadcasters Broadcasters
///   - Broadcasts TF transforms representing the relationship between map, odom, and robot frames.
///
/// @section parameters Parameters
///   `frequency (double) [default "95.0"]` - The frequency at which the node operates.
///   `body_id (string) [default "UNUSED"]` - The TF frame ID of the robot body.
///   `odom_id (string) [default "slam_odom"]` - The TF frame ID for the odometry frame.
///   `wheel_left (string) [default "UNUSED"]` - The TF frame ID of the left wheel.
///   `wheel_right (string) [default "UNUSED"]` - The TF frame ID of the right wheel.
///   `wheel_radius (double) [default "-1.0"]` - The radius of the robot's wheels.
///   `track_width (double) [default "-1.0"]` - The distance between the centers of the two wheels.
///   `motor_cmd_max (double) [default "-1.0"]` - The maximum command value for the motors.
///   `motor_cmd_per_rad_sec (double) [default "-1.0"]` - The command value per radian/second for the motors.
///   `encoder_ticks_per_rad (double) [default "-1.0"]` - The number of encoder ticks per radian of wheel rotation.

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
#include "nav_msgs/msg/path.hpp"

#include "turtlelib/diff_drive.hpp"
#include "turtlelib/slam.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker_array.hpp"

#include "nusim/srv/teleport.hpp"

using std::string;

using namespace std::chrono_literals;
using namespace turtlelib;

using std::placeholders::_1;
using std::placeholders::_2;

class nuslamNode : public rclcpp::Node
{
public:
    //
    // CONSTRUCTOR
    //
    nuslamNode() : Node("nuslam")
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
        declare_parameter("odom_id", "slam_odom", param_desc);
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
        // Additional variable initialization for diff_drive and slam components
        //
        init_var();

        //
        // SUBSCRIBERS
        //
        joint_states_sub = create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, std::bind(&nuslamNode::joint_states_callback, this, _1));
        // fake_sensor listener that implements slam w/ each callback
        fake_sensor_sub = create_subscription<visualization_msgs::msg::MarkerArray>("fake_sensor", 10, std::bind(&nuslamNode::fake_sensor_callback, this, _1));

        //
        // PUBLISHERS
        //
        odom_pub = create_publisher<nav_msgs::msg::Odometry>("odom", 100);
        // Publishing the path taken
        path_pub = create_publisher<nav_msgs::msg::Path>("nav_msgs/path", 100);
        // Publishing the estimate obstacle positions according to sensor data
        rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(50)).transient_local();
        obs_pub = create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", qos);

        //
        // SERVICE
        //
        initial_pose_srv = create_service<nusim::srv::Teleport>("initial_pose", std::bind(&nuslamNode::initial_pose_callback, this, _1, _2));

        //
        // BROADCASTER
        //
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        //
        // TIMER
        //
        timer = create_wall_timer(1.0s / frequency, std::bind(&nuslamNode::timer_callback, this));
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
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_sub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obs_pub;

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
    Slam slam_turtlebot;

    // Vectors
    std::vector<geometry_msgs::msg::PoseStamped> odom_path;

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

        //
        // JOINT STATE STUFF + CALCULATIONS + FIND ROBOT POS RELATIVE TO ODOM
        //
        // Note the new WheelPosition
        WheelPosition curr_wheel_position{latest_joint_states.position[1], latest_joint_states.position[0]};
        WheelPosition old_wheel_position = turtlebot.get_wheels();

        // Note the change in WheelPosition
        WheelPosition delta_wheel_position{calc_angle_diff(curr_wheel_position.right, old_wheel_position.right),
                                           calc_angle_diff(curr_wheel_position.left, old_wheel_position.left)};

        // Update the robot configuration with forward_k
        turtlebot.forward_k(delta_wheel_position);

        // Broadcast the TF from odom to body
        broadcast_tf_odom_robot(turtlebot.get_position().translation().x,
                    turtlebot.get_position().translation().y, turtlebot.get_position().rotation());

        //
        // SLAM STUFFF
        //
        slam_turtlebot.predict_and_update_xi(turtlebot.get_twist());
        // Broadcast the tf from world to odom to correct robot
        broadcast_tf_map_slam_odom();

        //
        // ODOM PATH STUFF
        //
        // Publish the corresponding odometry message
        // Initialize the odometry message
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = latest_joint_states.header.stamp;
        odom_msg.header.frame_id = "map";
        odom_msg.child_frame_id = body_id;

        // Fill in the positional information
        geometry_msgs::msg::Pose robot_pose;
        robot_pose.position.x = slam_turtlebot.get_transform().translation().x;
        robot_pose.position.y = slam_turtlebot.get_transform().translation().y;

        // Use a tf2 object and function to convert roll pitch yaw into quaternion
        tf2::Quaternion tf2_robot_quaternion;
        tf2_robot_quaternion.setRPY(0.0, 0.0, slam_turtlebot.get_transform().rotation());
        // tf2_robot_quaternion.normalize(); // Ensure the quaternion's magnitude is 1
        // Then convert that tf2 quaternion into a suitable quaternion message

        // geometry_msgs::msg::Quaternion robot_quaternion;
        // tf2::convert(tf2_robot_quaternion, robot_quaternion);
        robot_pose.orientation.x = tf2_robot_quaternion.x();
        robot_pose.orientation.y = tf2_robot_quaternion.y();
        robot_pose.orientation.z = tf2_robot_quaternion.z();
        robot_pose.orientation.w = tf2_robot_quaternion.w();

        // Retrieve the current twist of the robot and fill in twist message information
        Twist2D curr_twist = slam_turtlebot.twist();
        geometry_msgs::msg::Twist robot_twist;
        robot_twist.linear.x = curr_twist.x;
        robot_twist.linear.y = curr_twist.y;
        robot_twist.angular.z = curr_twist.omega;

        // Combine different components into the odometry message
        odom_msg.pose.pose = robot_pose;
        odom_msg.twist.twist = robot_twist;
        odom_pub->publish(odom_msg);

        // Apply the Pose information to a PoseStamped msg
        // and then append it to the odom_path vector
        store_posestamped(odom_msg);

        // Publish the path
        odom_path_publish();
    }

    void fake_sensor_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
        // Publish the obstacle positions according to sensor data
        visualization_msgs::msg::MarkerArray obs_array;

        for (size_t i = 0; i < msg->markers.size(); i++) {
            visualization_msgs::msg::Marker obs;
            obs.header.stamp = msg->markers[i].header.stamp;
            obs.header.frame_id = body_id;
            obs.id = i;
            obs.type = msg->markers[i].type;
            obs.action = msg->markers[i].action;

            obs.color.r = 0.0;
            obs.color.g = 1.0;
            obs.color.b = 0.0;
            obs.color.a = 1.0;

            obs.pose.position.x = msg->markers[i].pose.position.x;
            obs.pose.position.y = msg->markers[i].pose.position.y;
            obs.pose.position.z = msg->markers[i].pose.position.z;
            obs.scale.x = msg->markers[i].scale.x;
            obs.scale.y = msg->markers[i].scale.y;
            obs.scale.z = 0.2;
            obs_array.markers.push_back(obs);
        }

        obs_pub->publish(obs_array);

        //
        // SENSOR MEASUREMENT
        //
        visualization_msgs::msg::MarkerArray::SharedPtr sensed_features = msg;

        // Correct the robot's configuration for each sensed feature
        for (size_t i = 0; i < sensed_features->markers.size(); i++)
        {
            // Note the feature's position
            double x = sensed_features->markers[i].pose.position.x;
            double y = sensed_features->markers[i].pose.position.y;

            // Check if the object is detected by the robot
            if(sensed_features->markers[i].action == visualization_msgs::msg::Marker::ADD)
            {
                // Correct the robot's configuration with the sensed feature
                slam_turtlebot.correct_with_landmark(x, y, i);
            }
        }
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
        broadcast_tf_odom_robot(turtlebot.get_position().translation().x,
                    turtlebot.get_position().translation().y, turtlebot.get_position().rotation());

        // Give a successful response
        response->success = true;
    }

    //
    // TRANSFORM RELATED
    //
    // Broadcasts odom -> red robot transform
    void broadcast_tf_odom_robot(double x_in, double y_in, double theta_in)
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

    // Broadcasts map -> slam_odom transform
    void broadcast_tf_map_slam_odom()
    {
        // tf from world to slam robot position
        Transform2D tf_map_slam{slam_turtlebot.get_transform()};
        // tf from odom to robot (odom) position
        Transform2D tf_odom_robot{turtlebot.get_position()};
        // tf from world to odom (assume slam and odom are the same)
        Transform2D tf_map_odom = tf_map_slam * tf_odom_robot.inv();

        // Broadcast the tf from world to odom
        geometry_msgs::msg::TransformStamped msg;
        // Key info
        msg.header.stamp = latest_joint_states.header.stamp;
        msg.header.frame_id = "map";
        msg.child_frame_id = odom_id;

        // Translation
        msg.transform.translation.x = tf_map_odom.translation().x;
        msg.transform.translation.y = tf_map_odom.translation().y;

        // Rotation
        tf2::Quaternion raw_quat;
        raw_quat.setRPY(0, 0, tf_map_odom.rotation());
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

        // Initialize the SLAM object
        slam_turtlebot = Slam(turtlebot.get_position());
        // With q_t updated, update xi_t
        slam_turtlebot.update_xi();
        // Initialize the initial 'guess' values for sigma_t
        slam_turtlebot.initialize_sigma_t();
    }

    bool params_string_unfilled()
    {
        return (body_id == "UNUSED" || wheel_left == "UNUSED" || wheel_right == "UNUSED");
    }


    bool params_double_unfilled()
    {
        return (wheel_radius == -1.0 || motor_cmd_max == -1.0 || track_width == -1.0 || motor_cmd_per_rad_sec == -1.0 || encoder_ticks_per_rad == -1.0);
    }

    double calc_angle_diff(double subtracted, double subtractor)
    {
        double diff = subtracted - subtractor;
        if (diff > PI) {
            diff -= 2 * PI;
        } else if (diff < -PI) {
            diff += 2 * PI;
        }
        return diff;
    }

    void store_posestamped(nav_msgs::msg::Odometry odom_msg)
    {
        geometry_msgs::msg::PoseStamped pose_stamp;

        // Key info
        pose_stamp.header = odom_msg.header;

        // Position
        pose_stamp.pose.position.x = odom_msg.pose.pose.position.x;
        pose_stamp.pose.position.y = odom_msg.pose.pose.position.y;
        pose_stamp.pose.position.z = odom_msg.pose.pose.position.z;

        // Orientation
        pose_stamp.pose.orientation.x = odom_msg.pose.pose.orientation.x;
        pose_stamp.pose.orientation.y = odom_msg.pose.pose.orientation.y;
        pose_stamp.pose.orientation.z = odom_msg.pose.pose.orientation.z;
        pose_stamp.pose.orientation.w = odom_msg.pose.pose.orientation.w;

        // Append it to robot_path vector
        odom_path.push_back(pose_stamp);
    }

    void odom_path_publish()
    {
        nav_msgs::msg::Path msg;

        // Key info
        msg.header.stamp = rclcpp::Clock().now();
        msg.header.frame_id = "map";

        // Vector of Poses
        msg.poses = odom_path;

        // Publish the path trace
        path_pub->publish(msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<nuslamNode>());
    rclcpp::shutdown();
    return 0;
}