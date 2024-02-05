// export RCUTILS_CONSOLE_OUTPUT_FORMAT="[${severity}] [${time}] [${name}]: ${message}"
// general libraries
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>
#include <chrono>
#include <cstdlib>

class DistanceController : public rclcpp::Node
{
public:
    DistanceController()
        : Node("distance_controller_node"), MAX_LINEAR_SPEED_(0.8), MAX_ANGULAR_SPEED_(3.14),
        TIMER_MS_(25)
    {
        // create a subscriber
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered", 
            10, std::bind(&DistanceController::odomCallback, this, std::placeholders::_1));
        // create a publisher
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        // create a timer
        timer_ = this->create_wall_timer(std::chrono::milliseconds(TIMER_MS_), 
            std::bind(&DistanceController::timerCallback, this));
        timer_->cancel();
        // Declare parameters
        this->declare_parameter<float>("kp", 0.8);
        this->declare_parameter<float>("ki", 0.02);
        this->declare_parameter<float>("kd", 0.5);
        // Get parameters
        this->get_parameter("kp", kp_);
        this->get_parameter("ki", ki_);
        this->get_parameter("kd", kd_);
    }

    void setDesiredPosition(float desired_value)
    {
        desired_value_ = desired_value;
        odom_received_ = false;
        achieved_ = false;
    }

    bool hasReachedDesiredPosition()
    {
        return achieved_;
    }

    void robot_move(float linear_speed, float angular_speed)
    {
        geometry_msgs::msg::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = linear_speed;
        cmd_vel_msg.angular.z = angular_speed;
        cmd_vel_pub_->publish(cmd_vel_msg);
    }
private:
    // subscriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    float current_x_, current_y_, current_theta_;
    bool odom_received_ = false;
    // publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    const float MAX_LINEAR_SPEED_;
    const float MAX_ANGULAR_SPEED_;
    // timer
    rclcpp::TimerBase::SharedPtr timer_;
    const int TIMER_MS_ = 100;
    // control variables
    float tolerance_ = 0.02, desired_value_ = 0.0;
    float error_ = 0.0, previous_error_ = 0.0, integral_ = 0.0, derivative_ = 0.0;
    float kp_ = 0.5, ki_ = 0.0, kd_ = 0.0;
    bool achieved_ = false;
    float initial_pos_x_ = 0.0, initial_pos_y_ = 0.0;
    float desired_pos_x_ = 0.0, desired_pos_y_ = 0.0;

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Get the current distance from the odometry message
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        current_theta_ = quat2rpy(msg->pose.pose.orientation);
        // RCLCPP_INFO(this->get_logger(), "Odom values: x: '%.2f', y: '%.2f', theta: '%.2f'", current_x_, current_y_, current_theta_);
        if (!odom_received_)
        {
            initial_pos_x_ = current_x_; initial_pos_y_ = current_y_;
            desired_pos_x_ = initial_pos_x_ + desired_value_*cos(current_theta_);
            desired_pos_y_ = initial_pos_y_ + desired_value_*sin(current_theta_);
            error_ = sqrt(pow(desired_pos_x_ - current_x_, 2) + pow(desired_pos_y_ - current_y_, 2));
            previous_error_ = 0.0;
            integral_ = 0.0;
            achieved_ = false;
            // activate the timer
            timer_->reset();
            odom_received_ = true;
        }
    }

    double quat2rpy(const geometry_msgs::msg::Quaternion& quat)
    {
        // Convert the quaternion to a tf2 Quaternion
        tf2::Quaternion tf2_quat(quat.x, quat.y, quat.z, quat.w);

        // Convert the quaternion to a rotation matrix
        tf2::Matrix3x3 mat(tf2_quat);

        // Get the roll, pitch, and yaw from the rotation matrix
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);

        // Return the yaw
        return yaw;
    }

    template <typename T>
    T saturate(T var, T min, T max)
    {
        if (var > max)
        {
            return max;
        }
        else if (var < min)
        {
            return min;
        }
        else
        {
            return var;
        }
    }

    void timerCallback()
    {
        if (fabs(error_) > tolerance_)
        {
            control_algorithm_pid();
            // RCLCPP_INFO(this->get_logger(), "Error: '%.2f'", error_);
        }
        else
        {
            robot_move(0.0, 0.0);
            achieved_ = true;
            // deactivate the timer
            timer_->cancel();
        }
    }

    void control_algorithm_pid()
    {
        // calculate the error
        error_ = sqrt(pow(desired_pos_x_ - current_x_, 2) + pow(desired_pos_y_ - current_y_, 2));
        // determine the direction
        float direction = (desired_value_ < 0) ? -1.0 : 1.0;
        // proportional control
        float P = kp_ * error_;
        // integral control
        integral_ += error_ * (float)TIMER_MS_ / 1000.0;
        float I = ki_ * integral_;
        // derivative control
        derivative_ = (error_ - previous_error_) / ((float)TIMER_MS_ / 1000.0);
        float D = kd_ * derivative_;
        // control algorithm
        float control_signal = direction * (P + I + D);
        // update error
        previous_error_ = error_;
        // saturate the control
        control_signal = saturate(control_signal, -MAX_LINEAR_SPEED_, MAX_LINEAR_SPEED_);
        RCLCPP_INFO(this->get_logger(), "Error: '%.2f' - Control signal: '%.2f'", error_, control_signal);
        // move the robot
        robot_move(control_signal, 0.0);
    }

};

class TurnController : public rclcpp::Node
{
public:
    TurnController()
        : Node("turn_controller_node"), MAX_LINEAR_SPEED_(0.8), MAX_ANGULAR_SPEED_(3.14),
        TIMER_MS_(25)
    {
        // create a subscriber
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered", 
            10, std::bind(&TurnController::odomCallback, this, std::placeholders::_1));
        // create a publisher
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        // create a timer
        timer_ = this->create_wall_timer(std::chrono::milliseconds(TIMER_MS_), 
            std::bind(&TurnController::timerCallback, this));
        timer_->cancel();
        // Declare parameters
        this->declare_parameter<float>("kp", 2.3);
        this->declare_parameter<float>("ki", 0.0001);
        this->declare_parameter<float>("kd", 30.0);
        // Get parameters
        this->get_parameter("kp", kp_);
        this->get_parameter("ki", ki_);
        this->get_parameter("kd", kd_);
    }

    void setDesiredOrientation(float x_pos, float y_pos)
    {
        desired_value_ = atan2(y_pos, x_pos);
        odom_received_ = false;
        achieved_ = false;
    }

    bool hasReachedDesiredOrientation()
    {
        return achieved_;
    }

    void robot_move(float linear_speed, float angular_speed)
    {
        geometry_msgs::msg::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = linear_speed;
        cmd_vel_msg.angular.z = angular_speed;
        cmd_vel_pub_->publish(cmd_vel_msg);
    }
private:
    // subscriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    float current_x_, current_y_, current_theta_;
    bool odom_received_ = false;
    // publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    const float MAX_LINEAR_SPEED_;
    const float MAX_ANGULAR_SPEED_;
    // timer
    rclcpp::TimerBase::SharedPtr timer_;
    const int TIMER_MS_;
    // control variables
    float tolerance_ = 1.0*M_PI/180.0, desired_value_ = 0.0;
    float error_ = 0.0, previous_error_ = 0.0, integral_ = 0.0, derivative_ = 0.0;
    float kp_ = 0.5, ki_ = 0.0, kd_ = 0.0;
    bool achieved_ = false;
    float initial_angle_ = 0.0, desired_angle_ = 0.0;

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Get the current distance from the odometry message
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        current_theta_ = quat2rpy(msg->pose.pose.orientation);
        // RCLCPP_INFO(this->get_logger(), "Odom values: x: '%.2f', y: '%.2f', theta: '%.2f'", current_x_, current_y_, current_theta_);
        if (!odom_received_)
        {
            initial_angle_ = current_theta_;
            desired_angle_ = desired_value_; // if was relative angle, it must be added to the current angle!!!
            error_ = desired_angle_ - current_theta_;
            previous_error_ = 0.0;
            integral_ = 0.0;
            achieved_ = false;
            // activate the timer
            timer_->reset();
            odom_received_ = true;
        }
    }

    double quat2rpy(const geometry_msgs::msg::Quaternion& quat)
    {
        // Convert the quaternion to a tf2 Quaternion
        tf2::Quaternion tf2_quat(quat.x, quat.y, quat.z, quat.w);

        // Convert the quaternion to a rotation matrix
        tf2::Matrix3x3 mat(tf2_quat);

        // Get the roll, pitch, and yaw from the rotation matrix
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);

        // Return the yaw
        return yaw;
    }

    template <typename T>
    T saturate(T var, T min, T max)
    {
        if (var > max)
        {
            return max;
        }
        else if (var < min)
        {
            return min;
        }
        else
        {
            return var;
        }
    }

    void timerCallback()
    {
        static int within_tolerance_count = 0;
        const int tolerance_threshold = 10;  // Adjust this value as needed

        if (fabs(error_) > tolerance_)
        {
            control_algorithm_pid();
            within_tolerance_count = 0;  // Reset the counter
        }
        else
        {
            within_tolerance_count++;  // Increment the counter

            if (within_tolerance_count >= tolerance_threshold)
            {
                robot_move(0.0, 0.0);
                achieved_ = true;
                timer_->cancel();
            }
        }
    }

    void control_algorithm_pid()
    {
        // calculate the error
        error_ = desired_angle_ - current_theta_;
        // determine the shortest direction
        // float direction = 1.0;
        // if (error_ > M_PI)
        // {
        //     error_ = error_ - 2 * M_PI;
        // }
        // else if (error_ < -M_PI)
        // {
        //     error_ = error_ + 2 * M_PI;
        // }
        // if (error_ < 0)
        // {
        //     direction = -1.0;
        // }
        // proportional control
        float P = kp_ * error_;
        // integral control
        integral_ += error_ * (float)TIMER_MS_ / 1000.0;
        float I = ki_ * integral_;
        // derivative control
        derivative_ = (error_ - previous_error_) / ((float)TIMER_MS_ / 1000.0);
        float D = kd_ * derivative_;
        // control algorithm
        float control_signal = (P + I + D); // (P + I + D)*direction
        // update error
        previous_error_ = error_;
        // saturate the control
        control_signal = saturate(control_signal, -MAX_ANGULAR_SPEED_, MAX_ANGULAR_SPEED_);
        RCLCPP_INFO(this->get_logger(), "Error°: '%.2f' - Control signal: '%.2f'", error_*180.0/M_PI, control_signal);
        // move the robot
        robot_move(0.0, control_signal);
    }

};

int main(int argc, char **argv)
{
    setenv("RCUTILS_CONSOLE_OUTPUT_FORMAT", "[{severity}]: [{message}]", 1);
    rclcpp::init(argc, argv);
    // Check if a scene number argument is provided
    int scene_number_ = 1; // Default scene number
    if (argc > 1) {
        scene_number_ = std::atoi(argv[1]);
    }
    auto distance_controller = std::make_shared<DistanceController>();
    auto turn_controller = std::make_shared<TurnController>();
    // add node to the executor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(distance_controller);
    executor.add_node(turn_controller);

    // Declare waypoints before the switch statement
    std::vector<std::pair<double, double>> waypoints;
    // Create a list of waypoints, depending on the scene number
    switch (scene_number_) {
    case 1: // Simulation
        waypoints = {
        {0.0, 0.0},         // waypoint 1        
        {0.467, -0.017},    // waypoint 2       
        {0.471, -1.376},    // waypoint 3        
        {0.992, -1.408},    // waypoint 4        
        {1.025, -0.934},    // waypoint 5        
        {1.435, -0.917},    // waypoint 6        
        {1.446, -0.371},    // waypoint 7        
        {1.959, -0.363},    // waypoint 8        
        {1.954, 0.510},     // waypoint 9    
        {1.501, 0.497},     // waypoint 10    
        {1.491, 0.193},     // waypoint 11    
        {1.009, 0.174},     // waypoint 12   
        {0.583, 0.468},     // waypoint 13    
        {0.122, 0.481}      // waypoint 14   
        };
        break;

    case 2: // CyberWorld
        waypoints = {
        {-0.034, 0.151},    // waypoint 1                  
        {1.784, 0.290},     // waypoint 2              
        {1.853, -0.218},    // waypoint 3                  
        {1.444, -0.269},    // waypoint 4                  
        {1.508, -0.800},    // waypoint 5                  
        {1.918, -0.763},    // waypoint 6                  
        {2.076, -1.274},    // waypoint 7                  
        {1.702, -1.348},    // waypoint 8                  
        {1.724, -1.620},    // waypoint 9                  
        {1.186, -2.059},    // waypoint 10                 
        {1.076, -1.220},    // waypoint 11                 
        {0.599, -1.283},    // waypoint 12                 
        {0.537, -0.748},    // waypoint 13                 
        {0.107, -0.779}     // waypoint 14             
        };
        break;

    case 3: // Test World - calibration
        waypoints = {
        {1.0, 0.0},
        {0.0, 1.0}
        };
        break;

    default:
        RCLCPP_ERROR(turn_controller->get_logger(), "Invalid scene number: %d",scene_number_);
        return -1;
    }

    distance_controller->setDesiredPosition(1.0);
    while(!distance_controller->hasReachedDesiredPosition()){rclcpp::spin_some(distance_controller);}
    rclcpp::sleep_for(std::chrono::seconds(1));

    turn_controller->setDesiredOrientation(1.0, 1.0);
    while(!turn_controller->hasReachedDesiredOrientation()){rclcpp::spin_some(turn_controller);}
    rclcpp::sleep_for(std::chrono::seconds(1));

    distance_controller->setDesiredPosition(-1.0);
    while(!distance_controller->hasReachedDesiredPosition()){rclcpp::spin_some(distance_controller);}

    turn_controller->setDesiredOrientation(-1.0, -1.0);
    while(!turn_controller->hasReachedDesiredOrientation()){rclcpp::spin_some(turn_controller);}
    rclcpp::sleep_for(std::chrono::seconds(1));

    rclcpp::shutdown();
    return 0;
}