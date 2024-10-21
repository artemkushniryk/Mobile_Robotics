#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "tf2/utils.h"                             // getYaw
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // toMsg

class LaserProcessor : public rclcpp::Node
{
    // ----- Attributes -----
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr points_pub_;

    std_msgs::msg::ColorRGBA red;
    std_msgs::msg::ColorRGBA green;
    std_msgs::msg::ColorRGBA blue;
    double pose_x_, pose_y_, pose_yaw_;
    double sensor_x_, sensor_y_, sensor_z_, sensor_yaw_;

    // ----- Methods -----
public:
    // Constructor
    LaserProcessor()
        : Node("laser_processor")
    {
        // initialization
        red.r = 1.0;
        red.a = 0.5;
        green.g = 1.0;
        green.a = 0.5;
        blue.b = 1.0;
        blue.a = 0.5;

        // transform from odom to platform (automatically updated on odomCallback)
        pose_x_ = 0.0;
        pose_y_ = 0.0;
        pose_yaw_ = 0.0;

        // subscribers
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&LaserProcessor::laserCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&LaserProcessor::odomCallback, this, std::placeholders::_1));

        // publisher
        points_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("points", 1);

        // transform from platform to sensor
        // TODO 2a START
        // sensor_x_   =
        // sensor_y_   =
        // sensor_z_   =
        // sensor_yaw_ =
        // TODO 2a END
    }

    // Callback of laser scan
    void laserCallback(const sensor_msgs::msg::LaserScan &laser_msg)
    {
        int size = laser_msg.ranges.size();
        RCLCPP_INFO(this->get_logger(), "Received scan ranges size=%d", size);
        std::vector<double> x(size, 0.0);
        std::vector<double> y(size, 0.0);

        // print & visualize one of each n points
        int n_viz = 10;         // <-- CHANGE IT TO DISPLAY MORE OR LESS POINTS
        int n_print = size / 4; // <-- CHANGE IT TO DISPLAY MORE OR LESS POINTS

        // ======= C++ TIPS: Some examples of using std::vector =======

        // //Resize a vector
        // std::vector<int> v;
        // int my_size=2;
        // RCLCPP_INFO(this->get_logger(), "v.size()=%lu", v.size());
        // v.resize(my_size);
        // RCLCPP_INFO(this->get_logger(), "v.size()=%lu", v.size());
        // RCLCPP_INFO(this->get_logger(), "--");

        // //Add an element at the end
        // std::vector<double> vv;
        // double my_number=1.23;
        // double other_number=4.56;
        // RCLCPP_INFO(this->get_logger(), "vv.size()=%lu", vv.size());
        // vv.push_back(my_number);
        // RCLCPP_INFO(this->get_logger(), "vv.size()=%lu", vv.size());
        // vv.push_back(other_number);
        // RCLCPP_INFO(this->get_logger(), "vv.size()=%lu", vv.size());
        // RCLCPP_INFO(this->get_logger(), "--");

        // //Init vector with size and value
        // int other_size=3;
        // double value=7.89;
        // std::vector<double> vvv(other_size,value);
        // //Loop through a vector
        // for(unsigned int i=0; i<vvv.size(); i++)
        // {
        //   RCLCPP_INFO(this->get_logger(), "vvv[%d]=%f",i, vvv[i]);
        // }
        // RCLCPP_INFO(this->get_logger(), "--");

        // ======== TRANSFORMATION 1 ========
        // scan from polar to cartesian (sensor coordinates)

        // TODO 1 START
        //
        //
        //
        // TODO 1 END

        // OUTPUT1
        RCLCPP_INFO(this->get_logger(), "Cartesian (scan frame):");
        for (unsigned int i = 0; i < x.size(); i += n_print)
            RCLCPP_INFO(this->get_logger(), "for index=%d, x,y=%f,%f", i, x[i], y[i]);

        publishMarker(x, y, n_viz, "rplidar_link", "cartesian", 1, 0.0, red, laser_msg.header.stamp);

        // ======== TRANSFORMATION 2 ========
        // scan points in base_link coordinates

        // TODO 2b START
        //
        //
        //
        // TODO 2b END

        // OUTPUT2
        RCLCPP_INFO(this->get_logger(), "World frame:");
        for (unsigned int i = 0; i < x.size(); i += n_print)
            RCLCPP_INFO(this->get_logger(), "for index=%d, x,y=%f,%f", i, x[i], y[i]);

        publishMarker(x, y, n_viz, "base_link", "platform", 2, sensor_z_, blue, laser_msg.header.stamp);

        // ======== TRANSFORMATION 3 ========
        // scan points in odom coordinates

        // TODO 3 START
        //
        //
        //
        // TODO 3 END

        // OUTPUT2
        RCLCPP_INFO(this->get_logger(), "World frame:");
        for (unsigned int i = 0; i < x.size(); i += n_print)
            RCLCPP_INFO(this->get_logger(), "for index=%d, x,y=%f,%f", i, x[i], y[i]);

        publishMarker(x, y, n_viz, "odom", "global", 3, sensor_z_, green, laser_msg.header.stamp);

        RCLCPP_INFO(this->get_logger(), "---");
    }

    // Publisher of visualization markers for debugging purposes
    void publishMarker(const std::vector<double> &x,
                       const std::vector<double> &y,
                       const int &subsampling,
                       const std::string &frame_id,
                       const std::string &ns,
                       const int &id,
                       const double &z,
                       const std_msgs::msg::ColorRGBA &color,
                       const rclcpp::Time &t)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = t;
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::POINTS;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.01;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;
        marker.color = color;
        marker.lifetime = rclcpp::Duration(0, 0);

        geometry_msgs::msg::Point point;
        for (unsigned int i = 0; i < x.size(); i += subsampling)
        {
            if (x[i] == x[i] and y[i] == y[i] and abs(x[i]) < 1e3 and abs(y[i]) < 1e3)
            {
                point.x = x[i];
                point.y = y[i];
                point.z = z;
                marker.points.push_back(point);
            }
        }
        RCLCPP_INFO(this->get_logger(), "Publishing points of %s: %lu", ns.c_str(), marker.points.size());
        points_pub_->publish(marker);
    }

    // Callback of odometry
    void odomCallback(const nav_msgs::msg::Odometry &odom_msg)
    {
        pose_x_ = odom_msg.pose.pose.position.x;
        pose_y_ = odom_msg.pose.pose.position.y;
        pose_yaw_ = tf2::getYaw(odom_msg.pose.pose.orientation);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserProcessor>());
    rclcpp::shutdown();
    return 0;
}