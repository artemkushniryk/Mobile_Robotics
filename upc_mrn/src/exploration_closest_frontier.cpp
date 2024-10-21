#include "exploration_base.h"
#include <cmath>

class ExplorationClosestFrontier : public ExplorationBase
{
  protected:
    //////////////////////////////////////////////////////////////////////
    // TODO 1a: if you need your own attributes (variables) and/or
    //          methods (functions), define them HERE
    //////////////////////////////////////////////////////////////////////

    // // EXAMPLE ATTRIBUTE:
    double time_max_goal_ = 20; // max time to reach a goal before aborting
    double distance_limit_; // max distance to reach a goal before aborting
    double path_length;

    //////////////////////////////////////////////////////////////////////
    // TODO 1a END
    //////////////////////////////////////////////////////////////////////

  public:
    ExplorationClosestFrontier();

  protected:
    bool                replan() override;
    geometry_msgs::msg::Pose decideGoal() override;
};

ExplorationClosestFrontier::ExplorationClosestFrontier() : ExplorationBase("exploration_closest_frontier")
{
    //////////////////////////////////////////////////////////////////////
    // TODO 1b: You can set the value of attributes using ros param
    //          for changing the value without need of recompiling.
    //////////////////////////////////////////////////////////////////////

    // // EXAMPLE FOR LOADING PARAMS TO YOUR ATTRIBUTES:
    // // Get the value of the param "time_max_goal" and store it to the attribute 'time_max_goal_'.
    // // If the parameters is not defined, use the default value 20:
    //get_parameter_or("time_max_goal", time_max_goal_, 20);
    //get_parameter_or("distance_limit", distance_limit_, 5);
    //////////////////////////////////////////////////////////////////////
    // TODO 1b END
    //////////////////////////////////////////////////////////////////////
}

geometry_msgs::msg::Pose ExplorationClosestFrontier::decideGoal()
{
    ////////////////////////////////////////////////////////////////////
    // TODO 2: decide goal
    ////////////////////////////////////////////////////////////////////

    // // EXAMPLE iterating over detected frontiers
    // for (unsigned int i = 0; i < frontiers_msg_.frontiers.size(); i++)
    // {
    //   // Accessing different fields
    //   frontiers_msg_.frontiers[i].size;
    //   frontiers_msg_.frontiers[i].center_point.x;
    //   frontiers_msg_.frontiers[i].center_point.y;
    //   frontiers_msg_.frontiers[i].center_point.z;
    // }

    // // EXAMPLE filling Pose message
    // // The goal position can be filled with the center_point of the "best" frontier
    // g.position = frontiers_msg_.frontiers[i_best].center_free_point;
    //
    // // The orientation has to be filled as well.
    // g.orientation = robot_pose_.orientation;             // EXAMPLE1: the same orientation as the current one
    // g.orientation = tf::createQuaternionMsgFromYaw(0.0); // EXAMPLE2: zero yaw

    // // EXAMPLE check if a goal is valid and get path length to the goal
    // double path_length;
    // bool valid = isValidGoal(g, path_length)

    ////////////////////////////////////////////////////////////////////
    // TODO 2 END
    ////////////////////////////////////////////////////////////////////
    
    geometry_msgs::msg::Pose g;
    double min_distance = std::numeric_limits<double>::max();
    int i_closest = 0;
    
    for (unsigned int i = 0; i < frontiers_msg_.frontiers.size(); i++)
    {
      
      geometry_msgs::msg::Pose temp_pose;
      temp_pose.position = frontiers_msg_.frontiers[i].center_point;
      temp_pose.orientation = robot_pose_.orientation;
      g.position;
      
    if (isValidGoal(temp_pose, path_length) && min_distance > path_length)
    {
        min_distance = path_length;
        i_closest = i;
        g = temp_pose;
        
    }
  }
  // g.position = frontiers_msg_.frontiers[i_closest].center_point;
  // g.orientation = robot_pose_.orientation;
  return g;
}

bool ExplorationClosestFrontier::replan()
{
    // REMEMBER:
    // goal_time_ has the time since last goal was sent (seconds)
    // goal_distance_ has remaining distance to reach the last goal (meters)

    ////////////////////////////////////////////////////////////////////
    // TODO 3: replan
    ////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////
    // TODO 3 END
    ////////////////////////////////////////////////////////////////////

    // Replan ANYWAY if the robot reached or aborted the goal (DO NOT ERASE THE FOLLOWING LINES)
    // Define the radius around the goal to check
    // Define the radius around the goal to check
    // Update the map
    // Update the map  
    // Define the radius around the goal to check
    double time_last_goal = (this->get_clock()->now().seconds() - goal_time_);

    RCLCPP_INFO(this->get_logger(), "Goal distance: %f, Goal Time: %f", goal_distance_, goal_time_);

    // Replan ANYWAY if the robot reached or aborted the goal (DO NOT ERASE THE FOLLOWING LINES)
    if (robot_status_ != 0 || time_last_goal > time_max_goal_ || goal_distance_<= 0.5*path_length)
    {
        return true;
    }
    return false;

}

////// MAIN ////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  auto node = std::make_shared<ExplorationClosestFrontier>();
  exec.add_node(node);
  exec.spin();

  return 0;
}

