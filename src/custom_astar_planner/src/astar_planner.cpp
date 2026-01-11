#include "custom_astar_planner/astar_planner.hpp"
#include "custom_astar_planner/astar.hpp"

#include "pluginlib/class_list_macros.hpp"
#include <algorithm>
#include <cmath>

namespace custom_astar_planner
{

void AStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string,
  std::shared_ptr<tf2_ros::Buffer>,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  logger_ = node->get_logger();

  costmap_ros_ = costmap_ros.get();
  costmap_ = costmap_ros_->getCostmap();
}

double AStarPlanner::heuristic(int x1, int y1, int x2, int y2)
{
  return std::hypot(x1 - x2, y1 - y2);
}

nav_msgs::msg::Path AStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = costmap_ros_->getGlobalFrameID();
  path.header.stamp = rclcpp::Clock().now();

  unsigned int sx, sy, gx, gy;
  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, sx, sy) ||
      !costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, gx, gy))
  {
    RCLCPP_ERROR(logger_, "Start or goal outside costmap");
    return path;
  }

  std::priority_queue<Node, std::vector<Node>, Compare> open;
  std::unordered_map<int, Node> closed;

  int width = costmap_->getSizeInCellsX();

  Node start_node{(int)sx, (int)sy, 0.0,
                  heuristic(sx, sy, gx, gy), 0.0, -1, -1};
  start_node.f = start_node.g + start_node.h;
  open.push(start_node);

  std::vector<std::pair<int, int>> directions = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1},
    {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
  };

  Node goal_node;
  bool found = false;

  while (!open.empty())
  {
    Node current = open.top();
    open.pop();

    int key = current.y * width + current.x;
    if (closed.count(key))
      continue;

    closed[key] = current;

    if (current.x == (int)gx && current.y == (int)gy)
    {
      goal_node = current;
      found = true;
      break;
    }

    for (auto & d : directions)
    {
      int nx = current.x + d.first;
      int ny = current.y + d.second;

      if (nx < 0 || ny < 0 ||
          nx >= (int)costmap_->getSizeInCellsX() ||
          ny >= (int)costmap_->getSizeInCellsY())
        continue;

      if (costmap_->getCost(nx, ny) >= nav2_costmap_2d::LETHAL_OBSTACLE)
        continue;

      double move_cost =
        (std::abs(d.first) + std::abs(d.second) == 2) ? std::sqrt(2.0) : 1.0;

      Node next;
      next.x = nx;
      next.y = ny;
      next.g = current.g + move_cost;
      next.h = heuristic(nx, ny, gx, gy);
      next.f = next.g + next.h;
      next.px = current.x;
      next.py = current.y;

      open.push(next);
    }
  }

  if (!found)
  {
    RCLCPP_ERROR(logger_, "A* failed to find a path");
    return path;
  }

  Node current = goal_node;
  while (current.px != -1)
  {
    geometry_msgs::msg::PoseStamped pose;
    costmap_->mapToWorld(current.x, current.y,
                         pose.pose.position.x,
                         pose.pose.position.y);
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);

    int key = current.py * width + current.px;
    current = closed[key];
  }

  std::reverse(path.poses.begin(), path.poses.end());
  return path;
}

}  // namespace custom_astar_planner

PLUGINLIB_EXPORT_CLASS(
  custom_astar_planner::AStarPlanner,
  nav2_core::GlobalPlanner)
