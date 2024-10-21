#include "exploration_base.h"

class ExplorationBiggestFrontier : public ExplorationBase
{
  protected:
    //////////////////////////////////////////////////////////////////////
    // TODO 1a: if you need your own attributes (variables) and/or
    //          methods (functions), define them HERE
    //////////////////////////////////////////////////////////////////////

    // // EXAMPLE ATTRIBUTE:
    // double time_max_goal_; // max time to reach a goal before aborting

    //////////////////////////////////////////////////////////////////////
    // TODO 1a END
    //////////////////////////////////////////////////////////////////////
    double max_frontier_size_; // tamaño máximo de la frontera
    double path_length_;
    double time_max_goal_ = 45;

  public:
    ExplorationBiggestFrontier();

  protected:
    bool                replan() override;
    geometry_msgs::msg::Pose decideGoal() override;
};

ExplorationBiggestFrontier::ExplorationBiggestFrontier() : ExplorationBase("exploration_biggest_frontier")
{
    //////////////////////////////////////////////////////////////////////
    // TODO 1b: You can set the value of attributes using ros param
    //          for changing the value without need of recompiling.
    //////////////////////////////////////////////////////////////////////

    // // EXAMPLE FOR LOADING PARAMS TO YOUR ATTRIBUTES:
    // // Get the value of the param "time_max_goal" and store it to the attribute 'time_max_goal_'.
    // // If the parameters is not defined, use the default value 20:
    // get_parameter_or("time_max_goal", time_max_goal_, 20);

    //////////////////////////////////////////////////////////////////////
    // TODO 1b END
    //////////////////////////////////////////////////////////////////////
    get_parameter_or("max_frontier_size", max_frontier_size_, 20.0);
}

geometry_msgs::msg::Pose ExplorationBiggestFrontier::decideGoal()
{
    geometry_msgs::msg::Pose g;

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
    double max_size = 0.0;
    int i_big = 0;

    // TODO 2: decideGoal(): elige el nuevo objetivo
    for (unsigned int i = 0; i < frontiers_msg_.frontiers.size(); i++)
    {
        g.position = frontiers_msg_.frontiers[i].center_point;
        g.orientation = robot_pose_.orientation;

        bool valid = isValidGoal(g, path_length_);

        if (valid && frontiers_msg_.frontiers[i].size > max_size)
        {
            max_size = frontiers_msg_.frontiers[i].size;
            i_big = i;
        }
    }

    if (i_big != -1)
    {
        g.position = frontiers_msg_.frontiers[i_big].center_point;
        g.orientation = robot_pose_.orientation;
    }

    //     if (frontiers_msg_.frontiers[i].size > max_size)
    //     {
    //         max_size = frontiers_msg_.frontiers[i].size;
    //         i_best = i;
    //     }
    // }

    // if (i_best != -1)
    // {
    //     g.position = frontiers_msg_.frontiers[i_best].center_point;
    //     g.orientation = robot_pose_.orientation;
    // }

    return g;
}

bool ExplorationBiggestFrontier::replan()
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
    // TODO 3: replan(): decide si se debe decidir y enviar un nuevo objetivo al robot

    double path_length_covered = path_length_ - goal_distance_;

    RCLCPP_INFO(this->get_logger(), "Goal distance: %f, path_length_: %f", path_length_, path_length_covered);

    if (robot_status_ != 0 || goal_time_ > time_max_goal_|| path_length_covered >= 0.7 * path_length_) return true;

    return false;
}

////// MAIN ////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  auto node = std::make_shared<ExplorationBiggestFrontier>();
  exec.add_node(node);
  exec.spin();

  return 0;
}
