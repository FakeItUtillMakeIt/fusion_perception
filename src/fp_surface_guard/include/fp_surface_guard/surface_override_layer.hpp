#ifndef FP_SURFACE_GUARD__SURFACE_OVERRIDE_LAYER_HPP_
#define FP_SURFACE_GUARD__SURFACE_OVERRIDE_LAYER_HPP_

#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "fp_surface_guard/msg/surface_array.hpp"

namespace fp_surface_guard
{

/// GlassGuard 范式落地层：视觉提议 + 几何验证的表面，
/// 以"覆盖 free"的权限写入 costmap（排在 obstacle/static 之后、inflation 之前）。
class SurfaceOverrideLayer : public nav2_costmap_2d::Layer
{
public:
  SurfaceOverrideLayer();
  ~SurfaceOverrideLayer() override;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override {return false;}
  void deactivate() override;
  void activate() override;

protected:
  struct Surface
  {
    std::string class_name;
    rclcpp::Time stamp;                    // 最近一次被视觉重新提议的时刻
    std::vector<geometry_msgs::msg::Point> ring;   // costmap 全局系下的多边形
    unsigned char cost;
    double ttl;                            // 秒，过期后恢复所覆盖的 cell
    bool dirty {true};                     // 几何变化才重画(避免 restore->refill 闪烁)
    rclcpp::Time last_draw {0, 0, RCL_ROS_TIME};
    // 已写入的格子 (wx, wy, 写入前的旧代价)，用世界坐标记录以抗 rolling window 滚动
    std::vector<std::tuple<double, double, unsigned char>> marked;
  };

  void cb(const fp_surface_guard::msg::SurfaceArray::SharedPtr msg);
  int matchIndex(const Surface & cand) const;   // 同类邻近既有表面 => 索引, 否则 -1
  static bool sameGeometry(
    const std::vector<geometry_msgs::msg::Point> & a,
    const std::vector<geometry_msgs::msg::Point> & b);
  void expire(const rclcpp::Time & now, nav2_costmap_2d::Costmap2D * master);
  void restore(Surface & s, nav2_costmap_2d::Costmap2D * master);
  unsigned char costFor(const std::string & cls) const;
  double ttlFor(const std::string & cls) const;

  std::mutex mu_;
  std::vector<Surface> surfaces_;
  rclcpp::Subscription<fp_surface_guard::msg::SurfaceArray>::SharedPtr sub_;

  std::string topic_ {"/fp_guard/surfaces"};
  std::unordered_map<std::string, int> class_costs_;
  std::unordered_map<std::string, double> class_ttls_;
  int default_cost_ {150};
  double default_ttl_ {5.0};
  double min_confidence_ {0.3};
  double merge_dist_ {1.5};
  double refresh_period_ {2.0};            // 秒，滚动窗兜底周期性重画
  double max_ring_points_ {4000.0};
};

}  // namespace fp_surface_guard

#endif  // FP_SURFACE_GUARD__SURFACE_OVERRIDE_LAYER_HPP_
