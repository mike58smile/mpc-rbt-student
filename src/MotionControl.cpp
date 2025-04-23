#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "../include/MotionControl.hpp"
using namespace std::chrono_literals;

MotionControlNode::MotionControlNode() :
    rclcpp::Node("motion_control_node") {

        // Subscribers for odometry and laser scans
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&MotionControlNode::odomCallback, this, std::placeholders::_1)
        );
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&MotionControlNode::lidarCallback, this, std::placeholders::_1)
        );
        
        // Publisher for robot control
        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10
        ); 

        // Client for path planning
        plan_client_ = this->create_client<nav_msgs::srv::GetPlan>("/plan_path");

        // Action server
        nav_server_ = rclcpp_action::create_server<nav2_msgs::action::NavigateToPose>(
            this,
            "navigate_to_pose",
            std::bind(&MotionControlNode::navHandleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MotionControlNode::navHandleCancel, this, std::placeholders::_1),
            std::bind(&MotionControlNode::navHandleAccepted, this, std::placeholders::_1)
        );

        RCLCPP_INFO(get_logger(), "Motion control node started.");

        // Connect to path planning service server
        while (!plan_client_->wait_for_service(1s)) {
            RCLCPP_INFO(get_logger(), "Waiting for plan_path service...");
        }
        
        // // Init current_pose_
        current_pose_.header.frame_id = "map";
        current_pose_.pose.position.x = -0.5;
        current_pose_.pose.position.y = 0.0;
        current_pose_.pose.position.z = 0.095;
        current_pose_.pose.orientation.w = 1.0;
    }

void MotionControlNode::checkCollision() {
    // add code here

    // ********
    // * Help *
    // ********
    /*
    if (laser_scan_.ranges[i] < thresh) {
        geometry_msgs::msg::Twist stop;
        twist_publisher_->publish(stop);
    }
    */
}

void MotionControlNode::updateTwist() {
    // add code here

    // ********
    // * Help *
    // ********
    /*
    geometry_msgs::msg::Twist twist;
    twist.angular.z = P * xte;
    twist.linear.x = v_max;

    twist_publisher_->publish(twist);
    */
}

rclcpp_action::GoalResponse MotionControlNode::navHandleGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const nav2_msgs::action::NavigateToPose::Goal> goal) {
    (void)uuid; // Pokud UUID není použito
    (void)goal; // Pokud goal není použito
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; // Přijměte cíl
}

rclcpp_action::CancelResponse MotionControlNode::navHandleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
    (void)goal_handle; // Pokud goal_handle není použito
    return rclcpp_action::CancelResponse::ACCEPT; // Přijměte zrušení
}

void MotionControlNode::navHandleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
    // add code here
    goal_handle_ = goal_handle; // Uložení pro pozdější použití v execute()
    auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
    request->start = current_pose_; // nebo nastavte aktuální pozici robota
    request->goal = goal_handle->get_goal()->pose; // cíl z akce
    request->tolerance = 0.0;
    RCLCPP_INFO(this->get_logger(), "Requesting plan: start=(%.2f, %.2f), goal=(%.2f, %.2f)",
        request->start.pose.position.x, request->start.pose.position.y,
        request->goal.pose.position.x, request->goal.pose.position.y);
    RCLCPP_INFO(this->get_logger(), "Start frame: %s, Goal frame: %s",
        request->start.header.frame_id.c_str(), request->goal.header.frame_id.c_str());
    auto future = plan_client_->async_send_request(request,
        std::bind(&MotionControlNode::pathCallback, this, std::placeholders::_1));
}

void MotionControlNode::execute() {
    rclcpp::Rate loop_rate(5.0); // 5 Hz
    auto feedback = std::make_shared<nav2_msgs::action::NavigateToPose::Feedback>();

    while (rclcpp::ok()) {
        if (goal_handle_->is_canceling()) {
            goal_handle_->canceled(std::make_shared<nav2_msgs::action::NavigateToPose::Result>());
            RCLCPP_INFO(this->get_logger(), "Goal canceled.");
            return;
        }

        // Zde implementuj řízení podle path_ (např. zavolej updateTwist())

        // Posílej feedback
        goal_handle_->publish_feedback(feedback);

        // Pokud je cíl dosažen:
        // break;

        loop_rate.sleep();
    }

    // Po dosažení cíle:
    goal_handle_->succeed(std::make_shared<nav2_msgs::action::NavigateToPose::Result>());
    RCLCPP_INFO(this->get_logger(), "Goal succeeded.");
}

void MotionControlNode::pathCallback(rclcpp::Client<nav_msgs::srv::GetPlan>::SharedFuture future) {
    auto response = future.get();
    if (!response) {
        RCLCPP_ERROR(get_logger(), "Failed to receive path.");
        return;
    }
    if (response && response->plan.poses.size() > 0) {
        path_ = response->plan;
        //goal_handle_->execute();
        std::thread(&MotionControlNode::execute, this).detach();
        RCLCPP_INFO(this->get_logger(), "Path received with %zu points.", path_.poses.size());
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "Plánování trasy selhalo nebo je trasa prázdná.");
    }
}

void MotionControlNode::odomCallback(const nav_msgs::msg::Odometry & msg) {
    // add code here

    // ********
    // * Help *
    // ********
    /*
    checkCollision();
    updateTwist();
    */
   //current_pose_ = msg; // Ulož aktuální odometrii
}

void MotionControlNode::lidarCallback(const sensor_msgs::msg::LaserScan & msg) {
    // add code here
}
