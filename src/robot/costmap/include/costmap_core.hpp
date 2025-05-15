#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <vector>

namespace robot
{

struct CellIndex {
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex &other) const {
    return (x == other.x && y == other.y);
  }
};

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger, rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher);

    // core costmap methods
    void initializeCostmap();
    bool convertToGrid(double range, double angle, int& x_grid, int& y_grid);
    void markObstacle(int x_grid, int y_grid);
    void inflateObstacles();
    void publishCostmap(rclcpp::Time stamp);
    int getCost(int x, int y) const;

    int getWidth() const;
    int getHeight() const;
    double getResolution() const;

    double getOriginX() const { return origin_x_; }
    double getOriginY() const { return origin_y_; }

    void updateMap(const nav_msgs::msg::OccupancyGrid& map);

    CellIndex worldToMap(double wx, double wy);
    geometry_msgs::msg::Point mapToWorld(int mx, int my) const;
    bool inBounds(int x, int y) const;

  private:
    rclcpp::Logger logger_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;

    // internal costmap data
    std::vector<std::vector<int>> costmap_; // grid
    double origin_x_ = 0.0;
    double origin_y_ = 0.0;
    double resolution_ = 0.1;
    int width_ = 100;
    int height_ = 100;
    double inflation_radius_ = 1.0;
    int max_cost_ = 100;

    rclcpp::Time last_updated_;
};

}

#endif


// #ifndef COSTMAP_CORE_HPP_
// #define COSTMAP_CORE_HPP_

// #include "rclcpp/rclcpp.hpp"

// namespace robot
// {

// class CostmapCore {
//   public:
//     // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
//     explicit CostmapCore(const rclcpp::Logger& logger);

//   private:
//     rclcpp::Logger logger_;

// };

// }  

// #endif  