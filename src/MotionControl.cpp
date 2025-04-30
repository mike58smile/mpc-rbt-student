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
            "/tiago_base/Hokuyo_URG_04LX_UG01", 10,
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
// filepath: /home/student/stud/mpc_rbt_ws/src/mpc-rbt-student/src/MotionControl.cpp
    // Simple collision avoidance: stop if an obstacle is closer than thresh in front
    if (laser_scan_.ranges.empty()) return;

    const double thresh = 0.15; // [m] stop if obstacle is closer than this
    const double angle_window = 100.0 * M_PI / 180.0; // +/- 30 deg in front

    collision_detected_ = false;
    int n = laser_scan_.ranges.size();
    double angle = laser_scan_.angle_min;
    for (int i = 0; i < n; ++i, angle += laser_scan_.angle_increment) {
        // Only check points in front of the robot
        if (std::abs(angle) < angle_window) {
            if (laser_scan_.ranges[i] > laser_scan_.range_min &&
                laser_scan_.ranges[i] < thresh) {
                // Obstacle detected, stop the robot
                collision_detected_ = true;
                geometry_msgs::msg::Twist stop;
                twist_publisher_->publish(stop);
                RCLCPP_WARN(this->get_logger(), "Obstacle detected! Stopping.");
                break;
            }
        }
    }
}

void MotionControlNode::updateTwist() {
// filepath: /home/student/stud/mpc_rbt_ws/src/mpc-rbt-student/src/MotionControl.cpp
    checkCollision();
    if (!goal_handle_ || !goal_handle_->is_active() || path_.poses.empty() || collision_detected_) {
        geometry_msgs::msg::Twist stop;
        twist_publisher_->publish(stop);
        return;
    }

    // Parametry řízení
    const double v_max = 0.2; // m/s
    const double w_max = 0.8; // rad/s
    const double goal_tolerance = 0.1; // m
    const double waypoint_tolerance = 0.2; // m

    // Aktuální pozice robota
    const auto& pos = current_pose_.pose.position;
    const auto& ori = current_pose_.pose.orientation;
    double yaw = std::atan2(2.0 * (ori.w * ori.z + ori.x * ori.y),
                            1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z));

    // Odškrtávání bodů trasy
    while (path_.poses.size() > 1) {
        const auto& wp = path_.poses.front().pose.position;
        double dist = std::hypot(wp.x - pos.x, wp.y - pos.y);
        if (dist < waypoint_tolerance) {
            path_.poses.erase(path_.poses.begin());
        } else {
            break;
        }
    }

    // Sleduj aktuální cíl (první bod v trase)
    const auto& target = path_.poses.front().pose.position;
    double dx = target.x - pos.x;
    double dy = target.y - pos.y;
    double distance = std::hypot(dx, dy);
    double angle_to_goal = std::atan2(dy, dx);
    double angle_error = angle_to_goal - yaw;
    if (angle_error > M_PI) angle_error -= 2 * M_PI;
    if (angle_error < -M_PI) angle_error += 2 * M_PI;

    // Regulace
    double Kp_ang = 1.2;
    double Kp_lin = 0.8;

    geometry_msgs::msg::Twist twist;
    if (std::abs(angle_error) < 0.3) {
        twist.linear.x = v_max;
    } else {
        twist.linear.x = 0.0;
    }
    twist.angular.z = std::clamp(Kp_ang * angle_error, -w_max, w_max);

    // Pokud jsme u posledního bodu a blízko cíli, zastav
    if (path_.poses.size() == 1 && distance < goal_tolerance) {
        twist.linear.x = 0.0;
        twist.angular.z = 0.0;
    }

    twist_publisher_->publish(twist);
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
    rclcpp::Rate loop_rate(50.0); // 5 Hz
    auto feedback = std::make_shared<nav2_msgs::action::NavigateToPose::Feedback>();

    while (rclcpp::ok()) {
        if (goal_handle_->is_canceling()) {
            goal_handle_->canceled(std::make_shared<nav2_msgs::action::NavigateToPose::Result>());
            RCLCPP_INFO(this->get_logger(), "Goal canceled.");
            return;
        }

        // Abort if collision detected
        if (collision_detected_) {
            goal_handle_->abort(std::make_shared<nav2_msgs::action::NavigateToPose::Result>());
            RCLCPP_WARN(this->get_logger(), "Goal aborted due to obstacle.");
            return;
        }

        // Zde implementuj řízení podle path_ (např. zavolej updateTwist())
        updateTwist();
        // Posílej feedback
        goal_handle_->publish_feedback(feedback);

        if (path_.poses.size() == 1) {
            const auto& pos = current_pose_.pose.position;
            const auto& target = path_.poses.front().pose.position;
            double distance = std::hypot(target.x - pos.x, target.y - pos.y);
            if (distance < 0.1) break;
        }

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
    //RCLCPP_INFO(this->get_logger(), "odomCallback called: x=%.2f y=%.2f", msg.pose.pose.position.x, msg.pose.pose.position.y);
    // Ulož aktuální odometrii pro řízení
    current_pose_.header = msg.header;
    current_pose_.pose = msg.pose.pose;

    // Volitelně: kontrola kolizí (pokud implementováno)

    // Pokud je aktivní navigační akce, vypočítej a publikuj rychlosti
    // if (goal_handle_ && goal_handle_->is_active()) {
    //     updateTwist();
    // }
}

void MotionControlNode::lidarCallback(const sensor_msgs::msg::LaserScan & msg) {
    // add code here
    //RCLCPP_INFO(this->get_logger(), "lidarCallback called: angle_min=%.2f angle_max=%.2f", msg.angle_min, msg.angle_max);
    laser_scan_ = msg;
}
