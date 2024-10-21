#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class PubTwist : public rclcpp::Node
{
    // Attributes
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    size_t count_;

public:
    // Constructor
    PubTwist()
        : Node("pub_twist"), count_(0)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&PubTwist::publishTwistPeriodically, this));
    }

private:
    // Method that publish a twist message (called periodically by timer_)
    void publishTwistPeriodically()
    {
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = 0;
        twist_msg.angular.z = 1;

        RCLCPP_DEBUG(this->get_logger(), "Publishing twist message");

        publisher_->publish(twist_msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PubTwist>());
    rclcpp::shutdown();
    return 0;
}
