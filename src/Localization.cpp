#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "../include/Localization.hpp"
#include <chrono>
using namespace std::chrono_literals;

LocalizationNode::LocalizationNode() : 
    rclcpp::Node("localization_node"), 
    last_time_(this->get_clock()->now()) {

    // Odometry message initialization
    odometry_.header.frame_id = "map";
    odometry_.child_frame_id = "base_link";
    // add code here


    // Subscriber for joint_states
    joint_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&LocalizationNode::jointCallback, this, std::placeholders::_1)
    );

    // Publisher for odometry
    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/odom", 10
    );
    // timer_ = this->create_wall_timer(100ms, std::bind(&LocalizationNode::publishOdometry, this));

    // tf_briadcaster 
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(get_logger(), "Localization node started.");
}

void LocalizationNode::jointCallback(const sensor_msgs::msg::JointState & msg) {
    // add code here
    RCLCPP_INFO(this->get_logger(), "Received joint_states message with %zu positions", msg.position.size());

    auto current_time = this->get_clock()->now();
    auto dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    updateOdometry(msg.velocity[0], msg.velocity[1], dt);
    publishOdometry();
    publishTransform();
    RCLCPP_INFO(this->get_logger(), "Updated odometry and published transform.");
}

void LocalizationNode::updateOdometry(double left_wheel_vel, double right_wheel_vel, double dt) {

    double linear = robot_config::WHEEL_RADIUS * (left_wheel_vel + right_wheel_vel) / 2.0;
    double angular = robot_config::WHEEL_RADIUS * (right_wheel_vel - left_wheel_vel) / (robot_config::HALF_DISTANCE_BETWEEN_WHEELS * 2.0);


    tf2::Quaternion tf_quat;
    tf2::fromMsg(odometry_.pose.pose.orientation, tf_quat);
    double roll, pitch, theta;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, theta);

    theta = theta + angular * dt;
    theta = std::atan2(std::sin(theta), std::cos(theta));

    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
}

void LocalizationNode::publishOdometry() {
    // Fill the odometry message with dummy data for testing
    odometry_.header.stamp = this->get_clock()->now();
    odometry_.pose.pose.position.x += 0.1;  // Simulate movement in x
    odometry_.pose.pose.position.y += 0.0;  // No movement in y
    odometry_.pose.pose.orientation.z = 0.0;  // No rotation
    odometry_.pose.pose.orientation.w = 1.0;

    odometry_.twist.twist.linear.x = 0.1;  // Simulate linear velocity
    odometry_.twist.twist.angular.z = 0.0;  // No angular velocity

    // Publish the odometry message
    odometry_publisher_->publish(odometry_);

    RCLCPP_INFO(this->get_logger(), "Published odometry: x=%f, y=%f", 
                odometry_.pose.pose.position.x, odometry_.pose.pose.position.y);
}

void LocalizationNode::publishTransform() {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";         // Parent frame
    t.child_frame_id = "base_link";    // Child frame

    t.transform.translation.x = odometry_.pose.pose.position.x;
    t.transform.translation.y = odometry_.pose.pose.position.y;
    t.transform.translation.z = odometry_.pose.pose.position.z;
    t.transform.rotation = odometry_.pose.pose.orientation;

    tf_broadcaster_->sendTransform(t);
}
