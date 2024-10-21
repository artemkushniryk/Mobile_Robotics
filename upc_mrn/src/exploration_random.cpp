#include "exploration_base.h"

class ExplorationRandom : public ExplorationBase
{
protected:
  double goal_dist_threshold_; // replan if the goal is closer than goal_dist_threshold_

public:
  ExplorationRandom();

protected:
  bool replan() override;
  geometry_msgs::msg::Pose decideGoal() override;
};

ExplorationRandom::ExplorationRandom() : ExplorationBase("exploration_random")
{
  // Load parameters from ROS params
  get_parameter_or("goal_dist_threshold", goal_dist_threshold_, 0.5); // default 0.5m;

  // Start exploration
  exploration_started_ = true;
}

bool ExplorationRandom::replan()
{
  // Replan if the goal is closer than 'goal_dist_threshold_' m
  if (goal_distance_ < goal_dist_threshold_)
  {
    RCLCPP_INFO(this->get_logger(), "ExplorationRandom::replan(): Goal close enough REPLAN!");
    return true;
  }
  // Replan ANYWAY if the robot (reached or) aborted the goal
  if (robot_status_ != 0)
  {
    RCLCPP_INFO_EXPRESSION(this->get_logger(), robot_status_ == 1, "ExplorationRandom::replan(): Robot status reached REPLAN!");
    RCLCPP_WARN_EXPRESSION(this->get_logger(), robot_status_ == 2, "ExplorationRandom::replan(): Robot status aborted, REPLAN!");
    return true;
  }
  return false;
}

geometry_msgs::msg::Pose ExplorationRandom::decideGoal()
{
  // generate a random pose whitin the map (eventually will be invalid)
  geometry_msgs::msg::Pose g;
  g.position.x = map_.info.origin.position.x + map_.info.width * map_.info.resolution * (rand() % 100) / 100.0;
  g.position.y = map_.info.origin.position.y + map_.info.height * map_.info.resolution * (rand() % 100) / 100.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, 2 * M_PI * (rand() % 100) / 100.0);
  g.orientation = tf2::toMsg(q);

  return g;
}

////// MAIN ////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  auto node = std::make_shared<ExplorationRandom>();
  exec.add_node(node);
  exec.spin();

  return 0;
}
