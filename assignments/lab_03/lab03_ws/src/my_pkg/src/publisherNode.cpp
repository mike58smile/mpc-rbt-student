// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
using std::placeholders::_1;
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class PubSubNode : public rclcpp::Node
{
public:
  PubSubNode()
  : Node("myNode"), count_(0)
  {
    // Publisher definition
    publisher_ = this->create_publisher<std_msgs::msg::String>("node_name", 10);
    
    // Subscriber definition
    subscriber_ = this->create_subscription<std_msgs::msg::Float32>("battery_voltage",10, 			std::bind(&PubSubNode::topic_callback, this, _1));
    
    timer_ = this->create_wall_timer(
      500ms, std::bind(&PubSubNode::timer_callback, this));
  }

private:
  void topic_callback(const std_msgs::msg::Float32::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "%f", msg->data);
  }
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_;
  
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "myNode";
    RCLCPP_INFO(this->get_logger(), message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PubSubNode>());
  rclcpp::shutdown();
  return 0;
}
