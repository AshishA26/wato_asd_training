#include "planner_core.hpp"

namespace robot {

PlannerCore::PlannerCore(const rclcpp::Logger &logger) : logger_(logger) {
  path_ = std::make_shared<nav_msgs::msg::Path>();
  map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
}

void PlannerCore::planPath(const nav_msgs::msg::OccupancyGrid::SharedPtr &map,
                           const CellIndex &start, const CellIndex &goal) {

  map_ = map;
  path_->poses.clear();

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
  open_set.emplace(start, distance(start, goal)); // f_score for start node is 0

  // Note:
  // - G is the distance from starting node to current node
  // - H is the distance from current node to goal node (heuristic)
  // - F is the total cost of the node (F = G + H)

  // Loop until the open set is empty or we find the goal
  while (!open_set.empty()) {
    AStarNode current =
        open_set.top(); // Since min-heap, top is the node with lowest f_score
    open_set.pop();     // Remove the node from the open set
    closed_set.insert(current.index); // Add the node to the closed set

    if (current.index == goal) {
      // Path found! Reconstruct the path and store it in path_.
      // TODO
      return;
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
      // If neighbor is not traversable or neighbor is in closed set, skip it
      if (!isTraversable(neighbor) ||
          closed_set.find(neighbor) != closed_set.end()) {
        continue;
      }

      double new_g_score =
          g_scores[current.index] + distance(current.index, neighbor);

      // If neighbor is not in open set or new path to neighbor is shorter,
      // update the scores and parent. Note that since we don't check if the
      // neighbor is in the open set, we might add duplicates to the open set,
      // but they will be ignored when we pop them later if they have a higher
      // f_score.
      if (new_g_score < g_scores[neighbor] || !g_scores.count(neighbor)) {
        g_scores[neighbor] = new_g_score;
        came_from[neighbor] = current.index;
        double h_score = distance(neighbor, goal);
        double f_score = new_g_score + h_score;
        open_set.emplace(neighbor, f_score);
      }
    }
  }
}

bool PlannerCore::isTraversable(const CellIndex &idx) const {
  // Check if the index is within the bounds of the map
  if (idx.x < 0 || idx.y < 0 || idx.x >= static_cast<int>(map_->info.width) ||
      idx.y >= static_cast<int>(map_->info.height)) {
    return false; // Out of bounds
  }

  // Check if cell is occupied. Check the occupancy grid data at the
  // corresponding index. A value of -1 indicates an unknown cell, and values
  // from less than 50 indicate free space.
  int i =
      idx.y * map_->info.width + idx.x; // Get value from occupancy grid (1D)
  return map_->data[i] >= 0 && map_->data[i] < 50;
}

double PlannerCore::distance(const CellIndex &a, const CellIndex &b) const {
  // Use Euclidean distance
  return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

nav_msgs::msg::Path::SharedPtr PlannerCore::getPath() const { return path_; }

} // namespace robot
