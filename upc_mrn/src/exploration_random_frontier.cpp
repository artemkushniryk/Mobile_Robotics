#include "exploration_base.h"

class ExporationRandomFrontier : public ExplorationBase
{
protected:
    double dt_replan_, dist_goal_th_;

public:
    ExporationRandomFrontier();

protected:
    bool replan() override;
    geometry_msgs::msg::Pose decideGoal() override;
};

ExporationRandomFrontier::ExporationRandomFrontier() : ExplorationBase("exploration_random_frontier")
{
    // Load parameters from ROS params
    get_parameter_or("dt_replan", dt_replan_, 200.0);      // default 200s
    get_parameter_or("dist_goal_th", dist_goal_th_, 0.5); // default 0.5m
}

geometry_msgs::msg::Pose ExporationRandomFrontier::decideGoal()
{
    // Take a random valid frontier
    geometry_msgs::msg::Pose g;
    double path_length;
    auto i_rand = rand() % frontiers_msg_.frontiers.size();
    auto i_first = i_rand;
    g.position = frontiers_msg_.frontiers[i_rand].center_point;
    g.orientation = robot_pose_.orientation;

    // if not valid, iterate until valid (or reaching again the first index)
    while (not isValidGoal(g, path_length))
    {
        i_rand++;
        if (i_rand == frontiers_msg_.frontiers.size())
            i_rand = 0;
        if (i_rand == i_first)
            break;
        g.position = frontiers_msg_.frontiers[i_rand].center_point;
    }

    // it may eventually return a not valid goal if all are not valid
    return g;
}

bool ExporationRandomFrontier::replan()
{
    // Replan ANYWAY if the robot reached or aborted the goal
    if (robot_status_ != 0)
    {
        RCLCPP_INFO(this->get_logger(), "ExporationRandomFrontier::replan! robot_status_ %d", robot_status_);
        return true;
    }

    // time since last goal was sent
    if (goal_time_ > dt_replan_)
    {
        RCLCPP_INFO(this->get_logger(), "ExporationRandomFrontier::replan! Outdated goal %fs (max %fs)", goal_time_, dt_replan_);
        return true;
    }

    // distance to goal
    if (goal_distance_ < dist_goal_th_)
    {
        RCLCPP_INFO(this->get_logger(), "ExporationRandomFrontier::replan! Goal close enough %fm (min %fm)", goal_distance_, dist_goal_th_);
        return true;
    }

    return false;
}

////// MAIN ////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;
    auto node = std::make_shared<ExporationRandomFrontier>();
    exec.add_node(node);
    exec.spin();

    return 0;
}