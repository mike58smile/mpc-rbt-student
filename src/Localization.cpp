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

    odometry_.pose.pose.orientation.w = 1.0;
    odometry_.pose.pose.orientation.x = 0.0;
    odometry_.pose.pose.orientation.y = 0.0;
    odometry_.pose.pose.orientation.z = 0.0;

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
    const double L = robot_config::HALF_DISTANCE_BETWEEN_WHEELS * 2.0; // celková vzdialenosť medzi kolesami
    auto right_wheel_vel_lin = robot_config::WHEEL_RADIUS * right_wheel_vel; // lineárna rýchlosť pravého kolesa
    auto left_wheel_vel_lin = robot_config::WHEEL_RADIUS * left_wheel_vel; // lineárna rýchlosť ľavého kolesa
    
    // Výpočet lineárnej a uhlovej rýchlosti
    double linear_vel = (right_wheel_vel_lin + left_wheel_vel_lin) / 2.0;
    double angular_vel = (left_wheel_vel_lin - right_wheel_vel_lin) / L;

    // Získaj aktuálnu orientáciu (yaw = theta)
    tf2::Quaternion tf_quat;
    tf2::fromMsg(odometry_.pose.pose.orientation, tf_quat); // Konvertuj quaternion na tf2::Quaternion
    double roll, pitch, theta;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, theta);
    
    // Normalizuj theta
    theta = std::atan2(std::sin(theta), std::cos(theta));
    
    // Inkrementálna zmena
    double d_x = linear_vel * std::cos(theta) * dt;
    double d_y = linear_vel * std::sin(theta) * dt;
    double d_theta = angular_vel * dt;
    
    // Aktualizuj pozíciu a orientáciu
    odometry_.pose.pose.position.x += d_x;
    odometry_.pose.pose.position.y += d_y;
    theta += d_theta;
    
    // Previesť späť do kvaternionu
    tf2::Quaternion new_quat;
    new_quat.setRPY(0.0, 0.0, theta);
    odometry_.pose.pose.orientation = tf2::toMsg(new_quat);
    
    odometry_.twist.twist.linear.x = linear_vel;
    odometry_.twist.twist.angular.z = angular_vel;

}

void LocalizationNode::publishOdometry() {
    // Fill the odometry message with dummy data for testing
    odometry_.header.stamp = this->get_clock()->now();

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
