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

  double t_safe_, v_max_, robot_radius_;

  // ----- Methods -----
public:
  // Constructor
  SafetyNode()
      : Node("laser_processor")
  {
    // initialization
    v_max_ = 1e9;
 
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

  // Callback of laser scan
  void laserCallback(const sensor_msgs::msg::LaserScan &laser_msg)
  {
    // TODO 1: compute the maximum velocity allowed
     int size = laser_msg.ranges.size();// get range of laser msgs
     double rangemin = laser_msg.ranges[0]-robot_radius_; // inicial value of rangmin with first laser scan 

    for (int i = 1; i < size; i++) { // go to every laserscan 
     if (rangemin > laser_msg.ranges[i]-robot_radius_){ //  check if current is smaller
        rangemin = laser_msg.ranges[i]-robot_radius_; // update range min with current laser scan 
      }
    }
    v_max_ = rangemin/t_safe_;

    // END TODO 1
  }

  void cmdvelCallback(const geometry_msgs::msg::Twist &msg)
  {
  geometry_msgs::msg::Twist cmdvel_safe_msg = msg; // copy of the received message

  //TODO 2: modify cmdvel_safe_msg reducing its speed if necessary
  double linear_velocity = cmdvel_safe_msg.linear.x;// extract current information of linear velocity from topic 
  double angular_velocity = cmdvel_safe_msg.angular.z;// extract current information of angular velocity from topic 
  double vel_ratio; // initialize del velocity ratio

  if (abs(linear_velocity) > v_max_) { //check if the current velocity is bigger than the stablished max 
    RCLCPP_INFO(get_logger(), "Implementing SafetyNode! Max linear velocity = %f",v_max_);
    vel_ratio = v_max_/abs(linear_velocity); // calculate the ratio
    linear_velocity = vel_ratio*linear_velocity; // reduce by a ratio the linear velocity 
    angular_velocity = vel_ratio*angular_velocity; // reduce by a ratio the angular velocity
    }
  // create message to publish
  cmdvel_safe_msg.linear.x = linear_velocity; 
  cmdvel_safe_msg.angular.z = angular_velocity;
  // END TODO 2

  // publish the safe velocity command
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