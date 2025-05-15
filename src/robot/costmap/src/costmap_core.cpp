#include "costmap_core.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <cmath>
#include <vector>
#include <algorithm>

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger, rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher)
    : logger_(logger),
      publisher_(publisher),
      resolution_(0.1),
      width_(100),
      height_(100),
      inflation_radius_(1.0),
      max_cost_(100)
{
    initializeCostmap();
}

void CostmapCore::initializeCostmap() {
    costmap_ = std::vector<std::vector<int>>(height_, std::vector<int>(width_, 0));
}

bool CostmapCore::convertToGrid(double range, double angle, int& x_grid, int& y_grid) {
    double x = range * std::cos(angle);
    double y = range * std::sin(angle);

    x_grid = static_cast<int>(x / resolution_ + width_ / 2);
    y_grid = static_cast<int>(y / resolution_ + height_ / 2);

    return (x_grid >= 0 && x_grid < width_ && y_grid >= 0 && y_grid < height_);
}

void CostmapCore::markObstacle(int x_grid, int y_grid) {
    if (x_grid >= 0 && x_grid < width_ && y_grid >= 0 && y_grid < height_) {
        costmap_[y_grid][x_grid] = max_cost_;
    }
}

int CostmapCore::getCost(int x, int y) const {
    if (x >= 0 && x < width_ && y >= 0 && y < height_) {
        return costmap_[y][x];
    }
    return -1;
}

void CostmapCore::inflateObstacles() {
    std::vector<std::vector<int>> inflated = costmap_;
    int radius_cells = static_cast<int>(inflation_radius_ / resolution_);

    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            if (costmap_[y][x] == max_cost_) {
                for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
                    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
                        int nx = x + dx;
                        int ny = y + dy;

                        if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
                            double dist = std::sqrt(dx * dx + dy * dy) * resolution_;
                            if (dist <= inflation_radius_) {
                                int inflated_cost = static_cast<int>(max_cost_ * (1.0 - dist / inflation_radius_));
                                inflated_cost = std::clamp(inflated_cost, 0, max_cost_);
                                if (inflated_cost > inflated[ny][nx]) {
                                    inflated[ny][nx] = inflated_cost;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    costmap_ = inflated;
}

void CostmapCore::publishCostmap(rclcpp::Time stamp) {
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "map";

    msg.info.resolution = resolution_;
    msg.info.width = width_;
    msg.info.height = height_;
    msg.info.origin.position.x = -(width_ * resolution_) / 2.0;
    msg.info.origin.position.y = -(height_ * resolution_) / 2.0;
    msg.info.origin.position.z = 0.0;
    msg.info.origin.orientation.x = 0.0;
    msg.info.origin.orientation.y = 0.0;
    msg.info.origin.orientation.z = 0.0;
    msg.info.origin.orientation.w = 1.0;

    msg.data.resize(width_ * height_);
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            msg.data[y * width_ + x] = costmap_[y][x];
        }
    }

    RCLCPP_INFO(logger_, "Publishing costmap. Frame: %s, Size: %zu", msg.header.frame_id.c_str(), msg.data.size());
    publisher_->publish(msg);
}

void CostmapCore::updateMap(const nav_msgs::msg::OccupancyGrid& map) {
    if (map.info.width != static_cast<uint32_t>(width_) || static_cast<uint32_t>(height_)) {
        RCLCPP_WARN(logger_, "Received map with incompatible size! Resizing internal costmap...");
        width_ = map.info.width;
        height_ = map.info.height;
        initializeCostmap();
    }

    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            int cost = map.data[y * map.info.width + x];
            costmap_[y][x] = cost;  // -1 (unknown) is preserved
        }
    }

    RCLCPP_INFO(logger_, "Costmap updated with new map data.");
}

CellIndex CostmapCore::worldToMap(double wx, double wy) {
  int mx = static_cast<int>((wx - origin_x_) / resolution_);
  int my = static_cast<int>((wy - origin_y_) / resolution_);
  return CellIndex(mx, my);
}

geometry_msgs::msg::Point CostmapCore::mapToWorld(int mx, int my) const {
    geometry_msgs::msg::Point world_point;
    world_point.x = origin_x_ + (mx + 0.5) * resolution_;
    world_point.y = origin_y_ + (my + 0.5) * resolution_;
    world_point.z = 0.0;  // If 2D, z-coordinate will remain zero
    return world_point;
}

bool CostmapCore::inBounds(int x, int y) const {
    return x >= 0 && y >= 0 && x < getWidth() && y < getHeight();
}

int CostmapCore::getWidth() const {
    return width_;
}

int CostmapCore::getHeight() const {
    return height_;
}

}  
// namespace robot


