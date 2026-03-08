#include "costmap_core.hpp"

namespace robot {

CostmapCore::CostmapCore(const rclcpp::Logger &logger)
    : logger_(logger), resolution_(0.0), width_(0), height_(0) {}

void CostmapCore::initializeCostmap(double resolution, int width, int height,
                                    int8_t default_cell_value) {
  // Store the costmap parameters
  resolution_ = resolution;
  width_ = width;
  height_ = height;

  // Create a 2D array representing the OccupancyGrid. Each cell corresponds to
  // a grid space in the real world. Initialize all cells to a default value (0
  // for free space).
  occupancy_grid_.assign(height_,
                         std::vector<int8_t>(width_, default_cell_value));
}

void CostmapCore::convertToGrid(double range, double angle, int &x_grid,
                                int &y_grid) {
  // Compute the cartesian coordinates of the detected point (in meters)
  double x = range * cos(angle);
  double y = range * sin(angle);

  // Transform these coordinates into grid indices using the resolution and
  // origin of the costmap (origin is assumed to be at the center of the grid)
  x_grid = static_cast<int>(x / resolution_) + width_ / 2;
  y_grid = static_cast<int>(y / resolution_) + height_ / 2;
}

void CostmapCore::markObstacle(int x, int y, int8_t cost_occupied) {
  // Set the cells corresponding to detected obstacle positions to a high cost
  // value
  if (x >= 0 && x < width_ && y >= 0 && y < height_) {
    occupancy_grid_[y][x] = cost_occupied;
  }
}

void CostmapCore::inflateObstacles(double inflation_radius,
                                   int8_t max_cost_inflated_cell,
                                   int8_t cost_occupied) {
  // Create a copy of the occupancy grid to store inflated values
  std::vector<std::vector<int8_t>> inflated_grid = occupancy_grid_;

  // Convert inflation radius from meters to grid cells
  int radius = static_cast<int>(inflation_radius / resolution_);

  // Loop through each cell in the occupancy grid
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      // Skip cell if it is not an obstacle
      if (occupancy_grid_[y][x] != cost_occupied)
        continue;

      // Inflate surrounding cells within the inflation radius.
      // For each cell within the radius:
      // - Calculate the distance from the obstacle cell to the surrounding cell
      // - If the distance is within the radius, assign a cost to the
      // surrounding cell
      //   based on its dist to the obstacle cell (closer cells get higher
      //   cost).
      for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
          // Coordinates of the surrounding cell
          int nx = x + dx;
          int ny = y + dy;

          // Check if within grid bounds
          if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
            // Calculate Euclidean distance from the obstacle cell (in meters)
            double dist = std::sqrt(dx * dx + dy * dy) * resolution_;

            // If cell is within the inflation radius, then calculate the new
            // inflated cost and assign only if the calculated cost is higher
            // than the cell's current value
            if (dist <= inflation_radius) {
              int8_t inflated_cost = static_cast<int8_t>(
                  (1 - dist / inflation_radius) * max_cost_inflated_cell);
              inflated_grid[ny][nx] =
                  std::max(inflated_grid[ny][nx], inflated_cost);
            }
          }
        }
      }
    }
  }

  // Update the original occupancy grid with the inflated values
  occupancy_grid_ = inflated_grid;
}

std::vector<int8_t> CostmapCore::getCostmapData() const {
  // Return the 2D occupancy grid data as a 1D array
  std::vector<int8_t> data;
  data.reserve(width_ * height_); // Reserve space for efficiency
  for (const auto &row : occupancy_grid_) {
    data.insert(data.end(), row.begin(), row.end());
  }
  return data;
}

double CostmapCore::getResolution() const { return resolution_; }

int CostmapCore::getWidth() const { return width_; }

int CostmapCore::getHeight() const { return height_; }

} // namespace robot