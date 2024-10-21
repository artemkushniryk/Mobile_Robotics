#include "exploration_base.h"
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class ExplorationProject : public ExplorationBase
{
  protected:
    std::vector<geometry_msgs::msg::Pose> links_; // Store link frontiers
    geometry_msgs::msg::Pose centroid_; // Centroid of the biggest link frontier
    rclcpp::Time last_frontier_detection_time_; // Timestamp of the last new frontier detection
    geometry_msgs::msg::Pose previous_goal_; // Store the previous goal

    // Parameters
    double inflation_radius_;
    bool approaching_point_;
    bool new_frontier_detected_;
    double min_time_between_frontier_detection_;
    double goal_distance_;
    double replan_distance_threshold_;
    double time_max_goal_;
    double FRONTIER_MARKER_RADIUS = 1; // Placeholder value, replace with actual value
    bool is_goal_set_; // Indicates if a goal has been set

    // Publishers
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr links_markers_publisher_;

  public:
    ExplorationProject();

  protected:
    bool replan() override;
    geometry_msgs::msg::Pose decideGoal() override;
    geometry_msgs::msg::Pose calculateCentroid(const std::vector<geometry_msgs::msg::Pose>& poses);
    bool canReachPoint(const geometry_msgs::msg::Pose& point);
    void findLinkFrontiers();
    geometry_msgs::msg::Pose findClosestFrontier();
    geometry_msgs::msg::Pose findBiggestFrontierOfLink();
    bool isFrontierInList(const geometry_msgs::msg::Point& frontier, const std::vector<geometry_msgs::msg::Pose>& poses, double inflation_radius);
    void visualizeLinks();
};

ExplorationProject::ExplorationProject() : ExplorationBase("exploration_project")
{
    // Initialize parameters
    this->declare_parameter("inflation_radius", 1.5);
    this->declare_parameter("min_time_between_frontier_detection", 10.0); // Default: 10 seconds
    this->declare_parameter("replan_distance_threshold", 2.0); // Default: 1 meter
    this->declare_parameter("time_max_goal", 30.0); // Default: 60 seconds

    // Retrieve parameters
    this->get_parameter("inflation_radius", inflation_radius_);
    this->get_parameter("min_time_between_frontier_detection", min_time_between_frontier_detection_);
    this->get_parameter("replan_distance_threshold", replan_distance_threshold_);
    this->get_parameter("time_max_goal", time_max_goal_);

    // Initialize other attributes
    approaching_point_ = false;
    new_frontier_detected_ = false;
    previous_goal_ = geometry_msgs::msg::Pose();
    is_goal_set_ = false;

    // Create publisher for links markers
    links_markers_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("link_markers", 10);
}

geometry_msgs::msg::Pose ExplorationProject::decideGoal()
{
    geometry_msgs::msg::Pose g;

    // Step 1: Calculate link frontiers
    findLinkFrontiers();

    // Step 2: If no link found, get the closest frontier
    if (links_.empty())
    {
        g = findClosestFrontier();
    }
    else
    {
        // Step 3: Calculate centroid of link frontiers
        centroid_ = calculateCentroid(links_);

        // Step 4: Check if can go to centroid of the link
        if (is_goal_set_ && canReachPoint(centroid_))
        {
            g = centroid_;
        }
        else
        {
            // Step 5: Move to biggest frontier of the link
            g = findBiggestFrontierOfLink();
        }
    }

    previous_goal_ = g;
    is_goal_set_ = true;

    return g;
}

bool ExplorationProject::replan()
{
    // Replan if necessary, for example, when reaching the goal or other conditions

    // Replan if the robot reached or aborted the goal
    if (robot_status_ != 0)
    {
        RCLCPP_INFO(this->get_logger(), "robot status: %f",robot_status_);
        return true;
    }

    // Replan if the robot is approaching the point and a new frontier is detected
    if (approaching_point_ && new_frontier_detected_ == true)
    {
        // Check if enough time has passed since the last new frontier detection
        double time_since_last_frontier_detection = this->get_clock()->now().seconds() - last_frontier_detection_time_.seconds();
        if (time_since_last_frontier_detection >= min_time_between_frontier_detection_)
        {
            RCLCPP_INFO(this->get_logger(), "time_since_last_frontier_detection: %f",time_since_last_frontier_detection);
            return true;
            
        }
    }

    // Replan if the robot is approaching the point and the distance to the goal is less than a threshold
    if ("""approaching_point_ """ || goal_distance_ < replan_distance_threshold_)
    {   
        RCLCPP_INFO(this->get_logger(), "Replan por approaching_point_ && goal_distance_: %d", approaching_point_);
        return true;
        
    }

    // Replan if the time since the last goal was sent exceeds the maximum allowed time
    //double time_since_last_goal = this->get_clock()->now().seconds() - goal_time_;
    if (goal_time_ > time_max_goal_)
    {   
        RCLCPP_INFO(this->get_logger(), "Replan por time_since_last_goal: %f", goal_time_);
        return true;
        
    }

    // No need to replan otherwise
    return false;
}

void ExplorationProject::findLinkFrontiers()
{
    // Loop through frontiers and find link frontiers
    for (unsigned int i = 0; i < frontiers_msg_.frontiers.size(); i++)
    {
        geometry_msgs::msg::Point frontier_center = frontiers_msg_.frontiers[i].center_point;
        bool is_link = false;

        // Check if the frontier intersects with any other frontier within the inflation radius
        for (unsigned int j = 0; j < frontiers_msg_.frontiers.size(); j++)
        {
            if (i != j)
            {
                geometry_msgs::msg::Point other_frontier_center = frontiers_msg_.frontiers[j].center_point;
                double distance = std::hypot(frontier_center.x - other_frontier_center.x, frontier_center.y - other_frontier_center.y);
                double other_frontier_radius = FRONTIER_MARKER_RADIUS;
                if (distance <= inflation_radius_ + other_frontier_radius)
                {
                    is_link = true;
                    break;
                }
            }
        }

        // If the frontier is a link, add it to the links_ vector
        if (is_link && !isFrontierInList(frontier_center, links_, inflation_radius_))
        {
            geometry_msgs::msg::Pose link_pose;
            link_pose.position = frontier_center;
            link_pose.orientation = robot_pose_.orientation;
            links_.push_back(link_pose);
        }
    }
    this->visualizeLinks();
}

geometry_msgs::msg::Pose ExplorationProject::findClosestFrontier()
{
    geometry_msgs::msg::Pose closest_frontier;
    double min_distance = std::numeric_limits<double>::max();

    for (unsigned int i = 0; i < frontiers_msg_.frontiers.size(); i++)
    {
        geometry_msgs::msg::Point frontier_center = frontiers_msg_.frontiers[i].center_point;
        double distance = std::hypot(robot_pose_.position.x - frontier_center.x, robot_pose_.position.y - frontier_center.y);
        if (distance < min_distance)
        {
            min_distance = distance;
            closest_frontier.position = frontier_center;
            closest_frontier.orientation = robot_pose_.orientation;
        }
    }

    return closest_frontier;
}

geometry_msgs::msg::Pose ExplorationProject::findBiggestFrontierOfLink()
{
    geometry_msgs::msg::Pose biggest_frontier;
    double max_size = 0.0;

    for (const auto& link : links_)
    {
        for (unsigned int i = 0; i < frontiers_msg_.frontiers.size(); i++)
        {
            geometry_msgs::msg::Point frontier_center = frontiers_msg_.frontiers[i].center_point;
            double distance = std::hypot(link.position.x - frontier_center.x, link.position.y - frontier_center.y);
            if (distance <= inflation_radius_)
            {
                double size = frontiers_msg_.frontiers[i].size;
                if (size > max_size)
                {
                    max_size = size;
                    biggest_frontier.position = frontier_center;
                    biggest_frontier.orientation = link.orientation;
                }
            }
        }
    }

    return biggest_frontier;
}

geometry_msgs::msg::Pose ExplorationProject::calculateCentroid(const std::vector<geometry_msgs::msg::Pose>& poses)
{
    geometry_msgs::msg::Pose centroid;
    double sum_x = 0.0;
    double sum_y = 0.0;

    for (const auto& pose : poses)
    {
        sum_x += pose.position.x;
        sum_y += pose.position.y;
    }

    centroid.position.x = sum_x / poses.size();
    centroid.position.y = sum_y / poses.size();
    centroid.orientation = robot_pose_.orientation;

    return centroid;
}

bool ExplorationProject::canReachPoint(const geometry_msgs::msg::Pose& point)
{
    double distance_to_point = std::hypot(robot_pose_.position.x - point.position.x, robot_pose_.position.y - point.position.y);
    return distance_to_point <= goal_distance_;
}

bool ExplorationProject::isFrontierInList(const geometry_msgs::msg::Point& frontier, const std::vector<geometry_msgs::msg::Pose>& poses, double inflation_radius)
{
    for (const auto& pose : poses)
    {
        double distance = std::hypot(pose.position.x - frontier.x, pose.position.y - frontier.y);
        if (distance <= inflation_radius)
        {
            return true;
        }
    }
    return false;
}

void ExplorationProject::visualizeLinks()
{
    visualization_msgs::msg::MarkerArray markers;
    for (size_t i = 0; i < links_.size(); ++i)
    {
        const auto& link = links_[i];
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "links";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = link;
        marker.scale.x = 2 * inflation_radius_; // Diameter of the circle represented by the link
        marker.scale.y = 2 * inflation_radius_; // Diameter of the circle represented by the link
        marker.scale.z = 0.1; // Height of the cylinder
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.5; // Semi-transparent
        marker.lifetime = rclcpp::Duration::from_seconds(10.0); // Marker lifetime

        markers.markers.push_back(marker);
    }

    if (!links_markers_publisher_)
    {
        links_markers_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("link_markers", 10);
    }
    links_markers_publisher_->publish(markers);
    RCLCPP_INFO(this->get_logger(), "Se han publicado %zu marcadores.", markers.markers.size());
}

////// MAIN ////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;
    auto node = std::make_shared<ExplorationProject>();
    exec.add_node(node);
    exec.spin();

    return 0;
}
