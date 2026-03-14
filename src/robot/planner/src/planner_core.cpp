#include "planner_core.hpp"

namespace robot {

PlannerCore::PlannerCore(const rclcpp::Logger &logger) : logger_(logger) {}

nav_msgs::msg::Path
PlannerCore::planPath(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr &map,
                      const CellIndex &start, const CellIndex &goal,
                      const int cell_threshold) {

  nav_msgs::msg::Path path;

  // Open set (priority queue) and closed set (unordered set)
  // Open set is the set of nodes to explore while closed set is the set of
  // nodes already explored (only the indexes).
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  std::unordered_set<CellIndex, CellIndexHash> closed_set;

  // Maps to store the best known cost to reach each node and the parent of
  // each. came_from maps a node to the node it can most efficiently be reached
  // from. g_scores maps a node to the cost of the cheapest path from start to
  // that node.
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_map<CellIndex, double, CellIndexHash> g_scores;

  // Initialize the open set with the start node
  open_set.emplace(start, distance(start, goal));
  g_scores[start] = 0.0;

  // Note:
  // - G is the distance from starting node to current node
  // - H is the distance from current node to goal node (heuristic)
  // - F is the total cost of the node (F = G + H)

  // Loop until the open set is empty or we find the goal
  while (!open_set.empty()) {
    // Since min-heap, top is the node with lowest f_score
    AStarNode current = open_set.top();
    // Remove the node from the open set
    open_set.pop();
    if (closed_set.count(current.index)) {
      continue; // Skip if we have already processed this node
    }
    closed_set.insert(current.index); // Add the node to the closed set

    // Check if we have reached the goal. If so, reconstruct the path.
    if (current.index == goal) {
      CellIndex curr = goal;

      // While we have a parent for the current node, add it to the path and
      // move to the parent. Note need to convert from grid coordinates to world
      // coordinates when adding to the path.
      while (came_from.count(curr)) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x =
            curr.x * map->info.resolution + map->info.origin.position.x;
        pose.pose.position.y =
            curr.y * map->info.resolution + map->info.origin.position.y;
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
        curr = came_from[curr];
      }

      // Add the start node to the path
      geometry_msgs::msg::PoseStamped start_pose;
      start_pose.pose.position.x =
          start.x * map->info.resolution + map->info.origin.position.x;
      start_pose.pose.position.y =
          start.y * map->info.resolution + map->info.origin.position.y;
      path.poses.push_back(start_pose);

      // Reverse the path to get it from start to goal instead of goal to start
      std::reverse(path.poses.begin(), path.poses.end());

      return path;
    }

    // Generate neighbors of the current node
    CellIndex neighbors[8] = {
        CellIndex(current.index.x + 1, current.index.y),     // Right
        CellIndex(current.index.x - 1, current.index.y),     // Left
        CellIndex(current.index.x, current.index.y + 1),     // Up
        CellIndex(current.index.x, current.index.y - 1),     // Down
        CellIndex(current.index.x + 1, current.index.y + 1), // Up-Right
        CellIndex(current.index.x - 1, current.index.y + 1), // Up-Left
        CellIndex(current.index.x + 1, current.index.y - 1), // Down-Right
        CellIndex(current.index.x - 1, current.index.y - 1)  // Down-Left
    };

    // Check each neighbor
    for (const CellIndex &neighbor : neighbors) {
      const int dx = neighbor.x - current.index.x;
      const int dy = neighbor.y - current.index.y;

      // Prevent diagonal corner cutting.
      // If moving diagonally, both side-adjacent cells must be traversable.
      if (dx != 0 && dy != 0) {
        const CellIndex side_1(current.index.x + dx, current.index.y);
        const CellIndex side_2(current.index.x, current.index.y + dy);
        if (!isTraversable(side_1, map, cell_threshold) ||
            !isTraversable(side_2, map, cell_threshold)) {
          continue;
        }
      }

      // If neighbor is not traversable or neighbor is in closed set, skip it
      if (!isTraversable(neighbor, map, cell_threshold) ||
          closed_set.count(neighbor)) {
        continue;
      }

      // Calculate tentative g score for the neighbor. Note that due to
      // distance, a diagonal cell will have a slightly higher cost than a
      // straight cell, which aligns with the logic of using 1 and 1.414.
      double new_g_score =
          g_scores[current.index] + distance(current.index, neighbor);

      // If neighbor is not in g_scores or new path to neighbor is shorter,
      // update the scores and parent. Note that since we don't check if the
      // neighbor is in the open set, we might add duplicates to the open set,
      // but they will be ignored when we pop them later if they have a higher
      // f_score.
      if (!g_scores.count(neighbor) || new_g_score < g_scores[neighbor]) {
        g_scores[neighbor] = new_g_score;
        came_from[neighbor] = current.index;
        double h_score = distance(neighbor, goal);
        double f_score = new_g_score + h_score;
        open_set.emplace(neighbor, f_score);
      }
    }
  }
  RCLCPP_WARN(logger_, "Failed to find path");
  return path;
}

bool PlannerCore::isTraversable(
    const CellIndex &idx,
    const nav_msgs::msg::OccupancyGrid::ConstSharedPtr &map,
    const std::optional<int> cell_threshold) const {
  // Check if the index is within the bounds of the map
  if (idx.x < 0 || idx.y < 0 || idx.x >= static_cast<int>(map->info.width) ||
      idx.y >= static_cast<int>(map->info.height)) {
    return false; // Out of bounds
  }

  // If we are not checking cell cost, just return true since it's within bounds
  if (!cell_threshold.has_value()) {
    return true;
  }

  // Check if cell is occupied. Check the occupancy grid data at the
  // corresponding index. A value of -1 indicates an unknown cell, and values
  // less than cell_threshold indicate free space.
  int i = idx.y * map->info.width + idx.x; // Get value from occupancy grid (1D)
  return map->data[i] >= 0 && map->data[i] < cell_threshold;
}

double PlannerCore::distance(const CellIndex &a, const CellIndex &b) const {
  // Use Euclidean distance
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

CellIndex PlannerCore::convertWorldToGrid(
    const geometry_msgs::msg::Point &point,
    const nav_msgs::msg::OccupancyGrid::ConstSharedPtr &map) {
  // Converts world coordinates to grid indices based on the map's resolution
  // and origin. Need to shift the point relative to the map's origin and then
  // divide by the resolution to get cell indices.
  int x_index = static_cast<int>((point.x - map->info.origin.position.x) /
                                 map->info.resolution);
  int y_index = static_cast<int>((point.y - map->info.origin.position.y) /
                                 map->info.resolution);
  return CellIndex(x_index, y_index);
}

} // namespace robot
