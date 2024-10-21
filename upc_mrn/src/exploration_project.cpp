#include "exploration_base.h"
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class ExplorationProject : public ExplorationBase
{
  protected:
    std::vector<geometry_msgs::msg::Pose> links_; // Store link frontiers
    std::vector<geometry_msgs::msg::Pose> frontier_to_go_; // Store link frontiers
    geometry_msgs::msg::Pose centroid_; // Centroid of the biggest link frontier
    rclcpp::Time last_frontier_detection_time_; // Timestamp of the last new frontier detection
    geometry_msgs::msg::Pose previous_goal_; // Store the previous goal
    geometry_msgs::msg::Pose previous_previous_goal_; // Store the previous previous goal
    std::vector<double> link_distances_; // Store the distances of the link frontiers

    // Parameters
    double inflation_radius_;
    bool approaching_point_;
    bool new_frontier_detected_;
    double min_time_between_frontier_detection_;
    double replan_distance_threshold_;
    double time_max_goal_;
    double FRONTIER_MARKER_RADIUS = 1; // Placeholder value, replace with actual value
    bool is_goal_set_; // Indicates if a goal has been set
    double distance_travelled_;
    double w1_;
    double w2_;
    double similarity_threshold_;

    // Publishers
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr links_markers_publisher_;

  public:
    ExplorationProject();

  protected:
    bool replan() override;
    geometry_msgs::msg::Pose decideGoal() override;
    geometry_msgs::msg::Pose calculateCentroid(const std::vector<geometry_msgs::msg::Pose>& poses);
    void findLinkFrontiers();
    geometry_msgs::msg::Pose findClosestFrontier();
    bool isLinkInList(const geometry_msgs::msg::Pose& frontier, const std::vector<geometry_msgs::msg::Pose>& poses, double inflation_radius);
    void visualizeLinks();
    int estimation_of_goal(const std::vector<geometry_msgs::msg::Pose>& links, const geometry_msgs::msg::Pose& centroid, const std::vector<double>& link_distances);
};

ExplorationProject::ExplorationProject() : ExplorationBase("exploration_project")
{
    // Initialize parameters
    this->declare_parameter("inflation_radius", 1.0);
    this->declare_parameter("min_time_between_frontier_detection", 10.0); // Default: 10 seconds
    this->declare_parameter("replan_distance_threshold", 0.0); // Default: 1 meter
    this->declare_parameter("time_max_goal", 45.0); // Default: 60 seconds
    this->declare_parameter("w1", 0.8);
    this->declare_parameter("w2", 0.2);
    this->declare_parameter("similarity_threshold", 0.4);

    // Retrieve parameters
    this->get_parameter("inflation_radius", inflation_radius_);
    this->get_parameter("min_time_between_frontier_detection", min_time_between_frontier_detection_);
    this->get_parameter("replan_distance_threshold", replan_distance_threshold_);
    this->get_parameter("time_max_goal", time_max_goal_);
    this->get_parameter("w1", w1_); // Default: 1 meter
    this->get_parameter("w2", w2_); // Default: 60 seconds
    this->get_parameter("similarity_threshold", similarity_threshold_);

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
    int goal_to_reach_;
    geometry_msgs::msg::Pose temp_pose;
    double path_length;

    // Step 1: Calculate link frontiers
    findLinkFrontiers();
    
    if (links_.empty())
    {
        temp_pose = findClosestFrontier();
        if (isValidGoal(temp_pose, path_length)){
            double distance_to_previous_previous_goal = std::hypot(temp_pose.position.x - previous_previous_goal_.position.x, temp_pose.position.y - previous_previous_goal_.position.y);
            if (distance_to_previous_previous_goal < similarity_threshold_)
            {
                temp_pose = findClosestFrontier();
                if (isValidGoal(temp_pose, path_length))
                {
                    g = temp_pose;
                    replan_distance_threshold_ = 0.6*path_length;
                    RCLCPP_INFO(this->get_logger(), "Goal to reach is obtained through the closest frontier due to similarity with penultimate goal: Replan distance threshold is %f, and the goal is (x: %f, y: %f)", replan_distance_threshold_, g.position.x, g.position.y);
                }
            }
            else
            {
            g = temp_pose;
            replan_distance_threshold_ = 0.6*path_length;
            RCLCPP_INFO(this->get_logger(), "Goal to reach is obtained through the closest frontier: Replan distance threshold is %f, and the goal is (x: %f, y: %f)", replan_distance_threshold_, g.position.x, g.position.y);
            }
        }
    }
    else
    {
        // Step 2: Find the centroid of the link frontiers   
        centroid_ = calculateCentroid(links_);
    
        // Step 3: Estimate the goal to reach
        goal_to_reach_ = estimation_of_goal(links_, centroid_, link_distances_);  
    
        temp_pose = frontier_to_go_[goal_to_reach_];
        temp_pose.orientation = robot_pose_.orientation;

        // Step 4: Check if the goal is valid
        if (isValidGoal(temp_pose, path_length))
        {
            g = temp_pose;
            replan_distance_threshold_ = 0.6*path_length;
            RCLCPP_INFO(this->get_logger(), "Goal to reach is obtained through the link frontier: Replan distance threshold is %f, and the goal is (x: %f, y: %f)", replan_distance_threshold_, g.position.x, g.position.y);
        }
        else{
            temp_pose = findClosestFrontier();
            if (isValidGoal(temp_pose, path_length)){
                g = temp_pose;
                replan_distance_threshold_ = 0.6*path_length;
                RCLCPP_INFO(this->get_logger(), "Goal to reach is obtained through the closest frontier with link failure: Replan distance threshold is %f, and the goal is (x: %f, y: %f)", replan_distance_threshold_, g.position.x, g.position.y);
            }
        }
    }
    previous_previous_goal_ = previous_goal_;
    // Step 5: Store the goal as the previous goal
    previous_goal_ = g;

    
    // Step 6: Set the goal as the current goal
    return g;
}

bool ExplorationProject::replan()
{
    // Replan if necessary, for example, when reaching the goal or other conditions

    // Replan if the robot reached or aborted the goal
    if (robot_status_ != 0)
    {
        RCLCPP_INFO(this->get_logger(), "robot status: %d",robot_status_);
        return true;
    }

    // Replan if the robot is approaching the point and a new frontier is detected
    ///approaching_point_ &&
    // if (new_frontier_detected_ == true)
    // {
    //     // Check if enough time has passed since the last new frontier detection
    //     //last_frontier_detection_time_.seconds();///
    //     double time_since_last_frontier_detection =  goal_time_;
    //     if (time_since_last_frontier_detection >= min_time_between_frontier_detection_)
    //     {
    //         RCLCPP_INFO(this->get_logger(), "time_since_last_frontier_detection: %f",time_since_last_frontier_detection);
    //         return true;
            
    //     }
    // }

    // Replan if the robot is approaching the point and the distance to the goal is less than a threshold
    if (goal_distance_ < replan_distance_threshold_)
    {   
        RCLCPP_INFO(this->get_logger(), "Goal distance is less than the Replan Distance threshold: %f, %f", goal_distance_, replan_distance_threshold_);
        return true;
        
    }

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
    geometry_msgs::msg::Pose link_pose_center;// Store de point of the link
    double link_size = 0.0;
    double distance = 0.0;
    double other_frontier_radius = FRONTIER_MARKER_RADIUS;
    int frontier_length = 0;
    int other_frontier_length = 0;
    links_.clear();
    link_distances_.clear();
    frontier_to_go_.clear();

    // Loop through frontiers and find link frontiers
    for (unsigned int i = 0; i < frontiers_msg_.frontiers.size(); i++)
    {
        geometry_msgs::msg::Pose new_pose;
        geometry_msgs::msg::Point frontier_center = frontiers_msg_.frontiers[i].center_point; 
        frontier_length = frontiers_msg_.frontiers[i].size;

        // Check if the frontier intersects with any other frontier within the inflation radius
        for (unsigned int j = 0; j < frontiers_msg_.frontiers.size(); j++)
        {
            geometry_msgs::msg::Point other_frontier_center = frontiers_msg_.frontiers[j].center_point;
            other_frontier_length = frontiers_msg_.frontiers[j].size;
            
            if (i != j)
            {
                distance = std::hypot(frontier_center.x - other_frontier_center.x, frontier_center.y - other_frontier_center.y);
                
                if (distance <= inflation_radius_ + other_frontier_radius)
                {
                    link_pose_center.position.x = (frontier_center.x + other_frontier_center.x) / 2.0;
                    link_pose_center.position.y = (frontier_center.y + other_frontier_center.y) / 2.0;
                    link_pose_center.orientation = robot_pose_.orientation;
                    link_size = frontier_length + other_frontier_length;
                    if(isLinkInList(link_pose_center, links_, inflation_radius_) == false)
                    {
                        if(frontier_length >= other_frontier_length){
                            new_pose.position = frontiers_msg_.frontiers[i].center_point;
                            new_pose.orientation = robot_pose_.orientation;
                        }
                        else
                        {
                            new_pose.position = frontiers_msg_.frontiers[j].center_point;
                            new_pose.orientation = robot_pose_.orientation;
                        }
                        frontier_to_go_.push_back(new_pose);
                        std::cout << "Position: (" << new_pose.position.x << ", " << new_pose.position.y << ")\n";
                        links_.push_back(link_pose_center);
                        std::cout << "Position Center: (" << link_pose_center.position.x << ", " << link_pose_center.position.y << ")\n";
                        link_distances_.push_back(link_size);
                        break;
                    }
                }
            }
        }  
    }
    for (unsigned int i = 0; i < links_.size(); i++)
    {
        RCLCPP_INFO(this->get_logger(), "Link %d: (x: %f, y: %f)", i, links_[i].position.x, links_[i].position.y);
    }

    for (unsigned int i = 0; i < link_distances_.size(); i++)
    {
        RCLCPP_INFO(this->get_logger(), "Link distance %d: %f", i, link_distances_[i]);
    }

    for (unsigned int i = 0; i < frontier_to_go_.size(); i++)
    {
        RCLCPP_INFO(this->get_logger(), "Frontier to go %d: (x: %f, y: %f)", i, frontier_to_go_[i].position.x, frontier_to_go_[i].position.y);
    }                   
    this->visualizeLinks();
}

bool ExplorationProject::isLinkInList(const geometry_msgs::msg::Pose& new_link_frontier, const std::vector<geometry_msgs::msg::Pose>& links, double inflation_radius)
{
    for (const auto& pose : links)
    {
        double distance = std::hypot(pose.position.x - new_link_frontier.position.x, pose.position.y - new_link_frontier.position.y);
        if (distance <= inflation_radius)
        {
            return true;
        }
    }
    return false;
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

    // Imprimir el centroide
    std::cout << "Centroide: (" << centroid.position.x << ", " << centroid.position.y << ")" << std::endl;

    return centroid;
}

int ExplorationProject::estimation_of_goal(const std::vector<geometry_msgs::msg::Pose>& links, const geometry_msgs::msg::Pose& centroid, const std::vector<double>& link_distances)
{
    int optimal_point_index = 0;
    double max_density = -1.0;
    double lambda_1 = 0.05;
    double lambda_2 = 10;

    // Asegúrate de que links y link_distances tienen el mismo tamaño
    if (links.size() != link_distances.size())
    {
        RCLCPP_INFO(this->get_logger(), "Error with the size of links and link_distances");
        return optimal_point_index;
    }

    for (size_t i = 0; i < links.size(); ++i)
    {
        const auto& point = links[i];
        double frontier_length = lambda_2 * exp(-lambda_1 * link_distances[i]);

        // Al usar el inverso, los puntos con fronteras más cortas tienen un valor mayor, lo que también facilita la maximización.
        // Calcular la proximidad al centroide
        double proximity = 1.0 / (std::hypot(point.position.x - centroid.position.x, point.position.y - centroid.position.y));

        // Calcular la densidad
        double density = w1_ * proximity + w2_ * frontier_length;
        

        // Actualizar el punto óptimo si la densidad es mayor que la densidad máxima actual
        if (density > max_density)
        {
            max_density = density;
            optimal_point_index = i;
            RCLCPP_INFO(this->get_logger(), "The link with max densitiy is %d with the amount of %f", optimal_point_index, max_density);
        }
    }

    return optimal_point_index;
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
