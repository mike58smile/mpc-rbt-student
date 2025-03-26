#ifndef KEYBOARD_CONTROL_HPP
#define KEYBOARD_CONTROL_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <termios.h>

class KeyboardControlNode : public rclcpp::Node
{
public:
KeyboardControlNode();
    ~KeyboardControlNode();

private:

    void timer_callback();

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    struct termios old_termios_;
    geometry_msgs::msg::Twist twist_msg_; // Add this line

    double speed_;
};

#endif // KEYBOARD_CONTROL_HPP