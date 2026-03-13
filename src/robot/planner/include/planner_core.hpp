#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace robot {

// 2D grid index
struct CellIndex {
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex &other) const {
    return (x == other.x && y == other.y);
  }

  bool operator!=(const CellIndex &other) const {
    return (x != other.x || y != other.y);
  }
};

// Hash function for CellIndex so it can be used in std::unordered_map
struct CellIndexHash {
  std::size_t operator()(const CellIndex &idx) const {
    // A simple hash combining x and y
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

// Structure representing a node in the A* open set
struct AStarNode {
  CellIndex index;
  double f_score; // f = g + h

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

// Comparator for the priority queue (min-heap by f_score)
struct CompareF {
  bool operator()(const AStarNode &a, const AStarNode &b) {
    // We want the node with the smallest f_score on top
    return a.f_score > b.f_score;
  }
};

class PlannerCore {
public:
  explicit PlannerCore(const rclcpp::Logger &logger);

  // Main function to plan a path using A* algorithm
  void planPath(const nav_msgs::msg::OccupancyGrid::SharedPtr &map,
                const CellIndex &start, const CellIndex &goal);

  // Function to retrieve the planned path
  nav_msgs::msg::Path::SharedPtr getPath() const;

private:
  rclcpp::Logger logger_;
  nav_msgs::msg::Path::SharedPtr path_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;

  // Helper functions
  bool isTraversable(const CellIndex &idx) const;
  double distance(const CellIndex &a, const CellIndex &b) const;
};

} // namespace robot

#endif
