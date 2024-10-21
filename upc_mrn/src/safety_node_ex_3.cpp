// Artem Kushniryk Herasym
// Andres Castro Peñaranda

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class SafetyNode : public rclcpp::Node
{
  // ----- Attributes -----
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdvel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdvel_safe_pub_;
  geometry_msgs::msg::Twist cmdvel_safe_msg;
  geometry_msgs::msg::Twist last_cmdvel_;

  double t_safe_, v_max_, robot_radius_,v_max_back,v_max_front;

  // ----- Methods -----
public:
  // Constructor
  SafetyNode()
      : Node("laser_processor")
  {
    // initialization
    v_max_ = 1e9;
    v_max_back=1e9;
    v_max_front=1e9;
    // Declare parameters with default values
    declare_parameter("t_safe", 5.0);       // default 5s
    declare_parameter("robot_radius", 0.2); // default 20cm

    // load parameters
    get_parameter("t_safe", t_safe_);
    get_parameter("robot_radius", robot_radius_);

    RCLCPP_INFO(get_logger(), "SafetyNode constructor! Parameters: t_safe = %fs, robot_radius = %fs", t_safe_, robot_radius_);

    // publisher and subscribers
    cmdvel_safe_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&SafetyNode::laserCallback, this, std::placeholders::_1));
    cmdvel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel_teleop", 10, std::bind(&SafetyNode::cmdvelCallback, this, std::placeholders::_1));
  }

// Callback function for laser scan messages
void laserCallback(const sensor_msgs::msg::LaserScan &laser_msg){
    
    // Initialize minimum distance, angle, and projection to maximum possible values
    double min_distance_ = std::numeric_limits<double>::max();
    double angle_ = 0;
    double projection_ = std::numeric_limits<double>::max();
    v_max_= 1e9;

    // Get the angle resolution from the laser message
    double angle_resolution = laser_msg.angle_increment;

    // Udate minimum distance, angle, and projection
    auto update_min_distance = [&](double range, size_t i) {
        min_distance_ = range;
        angle_ = i * angle_resolution;
        projection_ = abs((min_distance_ + robot_radius_) * cos(angle_));
    };

    // Loop through all ranges in the laser message
    for (size_t i = 0; i < laser_msg.ranges.size(); ++i)
    {
      double range = laser_msg.ranges[i];
      
      // Check if the range is a valid number
      if (!std::isinf(range) && !std::isnan(range))
      {
        // Subtract the robot radius from the range
        range = range-robot_radius_;
        
        // If the range is less than the current minimum distance, update the minimum distance, angle, and projection
        if (range < min_distance_)
        {
          update_min_distance(range, i);
          
          // If the projection is less than 1.05 times the robot radius, update the maximum velocity
          if (projection_ < robot_radius_ * 1.05)
          {
            v_max_ = (i < laser_msg.ranges.size() / 2) ? min_distance_ / t_safe_ : -min_distance_ / t_safe_;
          }
        }
      }
   }
}

// Callback function for command velocity messages
void cmdvelCallback(const geometry_msgs::msg::Twist &msg)
{
    // Copy the received message
    geometry_msgs::msg::Twist cmdvel_safe_msg = msg;
   
    // Udate the safe command velocity message and print info
    auto update_velocity = [&](double v_max) {
        double scale_factor = v_max / cmdvel_safe_msg.linear.x;
        RCLCPP_INFO(get_logger(), "SafetyNode is active! Current velocity %f > max linear velocity = %f", cmdvel_safe_msg.linear.x, v_max);

        // Set the linear velocity to the maximum allowed
        cmdvel_safe_msg.linear.x = v_max;
        // Scale the angular velocity to maintain curvatures
        cmdvel_safe_msg.angular.z *= scale_factor;
    };

    // Check if the linear velocity is greater than the maximum allowed
    if (cmdvel_safe_msg.linear.x >= 0 && v_max_ >= 0 && cmdvel_safe_msg.linear.x > v_max_)
    {
        update_velocity(v_max_);
    }
    // Check if the linear velocity is less than the maximum allowed
    else if (cmdvel_safe_msg.linear.x < 0 && v_max_ < 0 && cmdvel_safe_msg.linear.x < v_max_)
    {
        update_velocity(v_max_);
    }

    // Publish the safe command velocity message
    cmdvel_safe_pub_->publish(cmdvel_safe_msg);
}

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyNode>());
  rclcpp::shutdown();
  return 0;
}