#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <vector>

class DistanceController : public rclcpp::Node
{
public:
    DistanceController(int scene_number = 1) : Node("distance_controller_node"), scene_number_(scene_number)
    {
        // PID gains - clearly defined values for distance control
        kp_ = 1.5;  // Proportional gain 
        ki_ = 0.1;  // Integral gain 
        kd_ = 0.8;  // Derivative gain 
        
        // Initialize waypoints based on scene
        SelectWaypoints();
        
        // Initialize control variables
        previous_error_ = 0.0;
        integral_ = 0.0;
        current_waypoint_index_ = 0;
        goal_reached_ = false;
        waypoint_reached_time_ = rclcpp::Time(0);
        first_odom_received_ = false;
        
        // Publishers and subscribers - FOR MECANUM DRIVE CONTROLLER
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/rosbot_xl_base_controller/odom", 10,
            std::bind(&DistanceController::odomCallback, this, std::placeholders::_1));
            
        // Control timer - 25 Hz for precise control
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(40),
            std::bind(&DistanceController::controlLoop, this));
            
        RCLCPP_INFO(this->get_logger(), "=== DISTANCE CONTROLLER INITIALIZED ===");
        RCLCPP_INFO(this->get_logger(), "PID Gains - Kp: %.2f, Ki: %.2f, Kd: %.2f", kp_, ki_, kd_);
        RCLCPP_INFO(this->get_logger(), "Scene: %d, Total waypoints: %zu", scene_number_, waypoints_.size());
        RCLCPP_INFO(this->get_logger(), "Using MECANUM_DRIVE_CONTROLLER for holonomic movement");
        RCLCPP_INFO(this->get_logger(), "Robot will move through waypoints without rotating");
    }

private:
    // PID gains - clearly defined and visible
    double kp_, ki_, kd_;
    
    // PID control variables
    double previous_error_;
    double integral_;
    
    // Robot state
    double current_x_, current_y_, current_yaw_;
    double start_x_, start_y_;
    double waypoint_start_x_, waypoint_start_y_; // Position when starting current waypoint
    bool goal_reached_;
    bool first_odom_received_;
    
    // Waypoint control
    std::vector<std::vector<double>> waypoints_;
    size_t current_waypoint_index_;
    int scene_number_;
    rclcpp::Time waypoint_reached_time_;
    
    // ROS components
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    void SelectWaypoints()
    {
        waypoints_ = {
            {0.0, 1.0, 0.0},     // waypoint 1: move 1m left from start
            {0.0, -1.0, 0.0},    // waypoint 2: move 1m right from waypoint 1 (back to start)
            {0.0, -1.0, 0.0},    // waypoint 3: move 1m right from start
            {0.0, 1.0, 0.0},     // waypoint 4: move 1m left from waypoint 3 (back to start)
            {1.0, 1.0, 0.0},     // waypoint 5: move 1m forward + 1m left from start
            {-1.0, -1.0, 0.0},   // waypoint 6: move 1m backward + 1m right from waypoint 5 (back to start)
            {1.0, -1.0, 0.0},    // waypoint 7: move 1m forward + 1m right from start
            {-1.0, 1.0, 0.0},    // waypoint 8: move 1m backward + 1m left from waypoint 7 (back to start)
            {1.0, 0.0, 0.0},     // waypoint 9: move 1m forward from start
            {-1.0, 0.0, 0.0}     // waypoint 10: move 1m backward from waypoint 9 (back to start)
        };
        
        RCLCPP_INFO(this->get_logger(), "Loaded %zu RELATIVE waypoints for scene %d", waypoints_.size(), scene_number_);
    }
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Update current position
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        
        // Convert quaternion to yaw
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, current_yaw_);
        
        // Set starting position on first callback
        if (!first_odom_received_) {
            start_x_ = current_x_;
            start_y_ = current_y_;
            waypoint_start_x_ = current_x_;
            waypoint_start_y_ = current_y_;
            first_odom_received_ = true;
            RCLCPP_INFO(this->get_logger(), "Starting position set: (%.3f, %.3f)", start_x_, start_y_);
            RCLCPP_INFO(this->get_logger(), "Beginning RELATIVE waypoint sequence...");
        }
    }
    
    void controlLoop()
    {
        if (!first_odom_received_ || goal_reached_) {
            return;
        }
        
        if (current_waypoint_index_ >= waypoints_.size()) {
            // All waypoints completed
            stopRobot();
            if (!goal_reached_) {
                goal_reached_ = true;
                RCLCPP_INFO(this->get_logger(), "All waypoints completed!");
                RCLCPP_INFO(this->get_logger(), "Distance controller program terminating successfully.");
            }
            return;
        }
        
        // Calculate target position - RELATIVE to waypoint start position
        double target_x = waypoint_start_x_ + waypoints_[current_waypoint_index_][0];
        double target_y = waypoint_start_y_ + waypoints_[current_waypoint_index_][1];
        
        // Calculate distance error to target
        double distance_error = calculateDistanceToTarget(target_x, target_y);
        
        // Check if current waypoint is reached
        if (distance_error < 0.05) { // 5cm tolerance for precision
            auto current_time = this->get_clock()->now();
            
            if (waypoint_reached_time_.seconds() == 0) {
                // Just reached waypoint
                waypoint_reached_time_ = current_time;
                RCLCPP_INFO(this->get_logger(), 
                           "Waypoint %zu Reached! Distance: %.3f m", 
                           current_waypoint_index_ + 1, distance_error);
                stopRobot();
            }
            
            // Stay stopped for 2 seconds at each waypoint
            if ((current_time - waypoint_reached_time_).seconds() >= 2.0) {
                // Move to next waypoint
                current_waypoint_index_++;
                waypoint_reached_time_ = rclcpp::Time(0);
                
                // Set new waypoint start position (current position)
                waypoint_start_x_ = current_x_;
                waypoint_start_y_ = current_y_;
                
                // Reset PID variables for new waypoint
                previous_error_ = 0.0;
                integral_ = 0.0;
                
                if (current_waypoint_index_ < waypoints_.size()) {
                    RCLCPP_INFO(this->get_logger(), 
                               "Moving to waypoint %zu: relative(%.2f, %.2f) from current position", 
                               current_waypoint_index_ + 1,
                               waypoints_[current_waypoint_index_][0],
                               waypoints_[current_waypoint_index_][1]);
                }
            }
            return;
        }
        
        // PID Control for distance
        double control_signal = calculatePIDControl(distance_error);
        
        // Calculate direction vector to target
        double direction_x = target_x - current_x_;
        double direction_y = target_y - current_y_;
        double distance = sqrt(direction_x * direction_x + direction_y * direction_y);
        
        if (distance > 0.01) {
            // Normalize direction vector
            direction_x /= distance;
            direction_y /= distance;
            
            // Create Twist message for MECANUM DRIVE CONTROLLER
            geometry_msgs::msg::Twist cmd_vel;
            
            // HOLONOMIC MOVEMENT: Robot moves in any direction while staying oriented forward
            cmd_vel.linear.x = control_signal * direction_x;  // Forward/backward movement
            cmd_vel.linear.y = control_signal * direction_y;  // Left/right (sideways) movement
            cmd_vel.linear.z = 0.0;
            cmd_vel.angular.x = 0.0;
            cmd_vel.angular.y = 0.0;
            cmd_vel.angular.z = 0.0;
            
            // Apply velocity limits for safety and good control
            double max_velocity = 0.8; // m/s - allow high speed for fast movement
            double current_speed = sqrt(cmd_vel.linear.x * cmd_vel.linear.x + 
                                      cmd_vel.linear.y * cmd_vel.linear.y);
            
            if (current_speed > max_velocity) {
                double scale_factor = max_velocity / current_speed;
                cmd_vel.linear.x *= scale_factor;
                cmd_vel.linear.y *= scale_factor;
            }
            
            // Publish velocity commands to mecanum drive controller
            cmd_vel_publisher_->publish(cmd_vel);
            
            // Debug output every second
            static int debug_counter = 0;
            if (debug_counter++ % 25 == 0) {
                RCLCPP_INFO(this->get_logger(), 
                           "Waypoint%zu: dist=%.3fm, ctrl=%.3f, vel=(%.3f,%.3f), pos=(%.3f,%.3f)->target(%.3f,%.3f)",
                           current_waypoint_index_ + 1, distance_error, control_signal,
                           cmd_vel.linear.x, cmd_vel.linear.y,
                           current_x_, current_y_, target_x, target_y);
            }
        }
    }
    
    double calculateDistanceToTarget(double target_x, double target_y)
    {
        double dx = target_x - current_x_;
        double dy = target_y - current_y_;
        return sqrt(dx * dx + dy * dy);
    }
    
    double calculatePIDControl(double error)
    {
        // PID Controller implementation with all three terms
        double dt = 0.04; // 40ms control loop period
        
        // Proportional term
        double proportional = kp_ * error;
        
        // Integral term with anti-windup
        integral_ += error * dt;
        // Clamp integral to prevent windup
        double max_integral = 0.5;
        if (integral_ > max_integral) integral_ = max_integral;
        if (integral_ < -max_integral) integral_ = -max_integral;
        double integral_term = ki_ * integral_;
        
        // Derivative term
        double derivative = kd_ * (error - previous_error_) / dt;
        previous_error_ = error;
        
        // Combined PID control signal
        double control_signal = proportional + integral_term + derivative;
        
        // Limit control signal output
        double max_control = 1.0;
        if (control_signal > max_control) control_signal = max_control;
        if (control_signal < 0.0) control_signal = 0.0; // Distance control is always positive
        
        return control_signal;
    }
    
    void stopRobot()
    {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.linear.z = 0.0;
        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = 0.0;
        cmd_vel.angular.z = 0.0;
        cmd_vel_publisher_->publish(cmd_vel);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    // Check if a scene number argument is provided
    int scene_number = 1; // Default to simulation
    if (argc > 1) {
        scene_number = std::atoi(argv[1]);
    }
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting Distance Controller for Scene %d", scene_number);
    
    auto node = std::make_shared<DistanceController>(scene_number);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}