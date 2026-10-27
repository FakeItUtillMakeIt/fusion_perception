#include "fp_surface_guard/surface_override_layer.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "tf2/time.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/convert.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace fp_surface_guard
{

using nav2_costmap_2d::Costmap2D;

namespace
{

bool pointInRing(const std::vector<geometry_msgs::msg::Point> & ring, double x, double y)
{
  // 奇偶射线法
  bool inside = false;
  const size_t n = ring.size();
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    const double xi = ring[i].x, yi = ring[i].y;
    const double xj = ring[j].x, yj = ring[j].y;
    if (((yi > y) != (yj > y)) &&
      (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi))
    {
      inside = !inside;
    }
  }
  return inside;
}

}  // namespace

SurfaceOverrideLayer::SurfaceOverrideLayer()
{
  class_costs_ = {{"glass", 100}, {"water", 65}, {"stair_down", 100}};
  class_ttls_ = {{"glass", 10.0}, {"water", 5.0}, {"stair_down", 10.0}};
}

SurfaceOverrideLayer::~SurfaceOverrideLayer()
{
  reset();
}

void SurfaceOverrideLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  declareParameter("topic", rclcpp::ParameterValue(std::string("/fp_guard/surfaces")));
  declareParameter("default_cost", rclcpp::ParameterValue(65));
  declareParameter("default_ttl", rclcpp::ParameterValue(5.0));
  declareParameter("min_confidence", rclcpp::ParameterValue(0.3));
  declareParameter("merge_dist", rclcpp::ParameterValue(1.5));
  declareParameter("refresh_period", rclcpp::ParameterValue(2.0));
  declareParameter("class_names", rclcpp::ParameterValue(std::vector<std::string>{}));
  declareParameter("class_costs", rclcpp::ParameterValue(std::vector<int64_t>{}));
  declareParameter("class_ttls", rclcpp::ParameterValue(std::vector<double>{}));

  node->get_parameter(getFullName("topic"), topic_);
  node->get_parameter(getFullName("default_cost"), default_cost_);
  node->get_parameter(getFullName("default_ttl"), default_ttl_);
  node->get_parameter(getFullName("min_confidence"), min_confidence_);
  node->get_parameter(getFullName("merge_dist"), merge_dist_);
  node->get_parameter(getFullName("refresh_period"), refresh_period_);
  std::vector<std::string> names;
  std::vector<int64_t> costs;
  std::vector<double> ttls;
  node->get_parameter(getFullName("class_names"), names);
  node->get_parameter(getFullName("class_costs"), costs);
  node->get_parameter(getFullName("class_ttls"), ttls);
  for (size_t i = 0; i < names.size(); i++) {
    if (i < costs.size()) {
      class_costs_[names[i]] = static_cast<int>(costs[i]);
    }
    if (i < ttls.size()) {
      class_ttls_[names[i]] = ttls[i];
    }
  }

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  sub_ = node->create_subscription<fp_surface_guard::msg::SurfaceArray>(
    topic_, rclcpp::QoS(10),
    [this](fp_surface_guard::msg::SurfaceArray::SharedPtr msg) {cb(msg);},
    options);
  RCLCPP_INFO(logger_, "SurfaceOverrideLayer on '%s' classes=%d", topic_.c_str(),
    static_cast<int>(class_costs_.size()));
}

unsigned char SurfaceOverrideLayer::costFor(const std::string & cls) const
{
  auto it = class_costs_.find(cls);
  int c = (it != class_costs_.end()) ? it->second : default_cost_;
  if (c < 0) {return 0;}  // 负数 = 丢弃
  return static_cast<unsigned char>(std::min(c, 254));
}

double SurfaceOverrideLayer::ttlFor(const std::string & cls) const
{
  auto it = class_ttls_.find(cls);
  return (it != class_ttls_.end()) ? it->second : default_ttl_;
}

void SurfaceOverrideLayer::cb(const fp_surface_guard::msg::SurfaceArray::SharedPtr msg)
{
  const std::string global_frame = layered_costmap_->getGlobalFrameID();
  const rclcpp::Time stamp(msg->header.stamp, RCL_ROS_TIME);

  std::vector<Surface> incoming;
  for (const auto & s : msg->surfaces) {
    if (s.confidence < min_confidence_ || s.polygon.points.size() < 3) {
      continue;
    }
    const int cost = costFor(s.class_name);
    if (cost == 0) {continue;}
    Surface sur;
    sur.class_name = s.class_name;
    sur.cost = static_cast<unsigned char>(cost);
    sur.ttl = ttlFor(s.class_name);
    sur.stamp = (s.header.stamp.sec || s.header.stamp.nanosec) ?
      rclcpp::Time(s.header.stamp, RCL_ROS_TIME) : stamp;
    // 取一次"最新"变换(TimePointZero)应用到整多边形：
    // 表面由 builder 每帧重投，单周期内位移可忽略；避免用观测时间戳超前于
    // map->odom 最新 TF 而整帧丢弃（全局 costmap 的 base_link 表面此前正是如此失效）。
    tf2::Transform tf_in_to_global;
    bool need_tf = !s.header.frame_id.empty() && s.header.frame_id != global_frame;
    if (need_tf) {
      try {
        auto hs = tf_->lookupTransform(global_frame, s.header.frame_id,
            tf2::TimePointZero, tf2::durationFromSec(0.2));
        tf2::fromMsg(hs.transform, tf_in_to_global);
      } catch (const tf2::TransformException &) {
        return;  // TF 未就绪，整帧丢弃
      }
    }
    for (const auto & p32 : s.polygon.points) {
      geometry_msgs::msg::Point p;
      if (need_tf) {
        tf2::Vector3 vin(p32.x, p32.y, p32.z);
        tf2::Vector3 vg = tf_in_to_global * vin;
        p.x = vg.x(); p.y = vg.y(); p.z = 0.0;
      } else {
        p.x = p32.x; p.y = p32.y; p.z = 0.0;
      }
      sur.ring.push_back(p);
    }
    incoming.push_back(std::move(sur));
  }

  std::lock_guard<std::mutex> lk(mu_);
  std::vector<Surface> next;
  std::vector<bool> consumed(surfaces_.size(), false);
  for (auto & cand : incoming) {
    int idx = matchIndex(cand);
    if (idx >= 0) {
      const auto & old = surfaces_[idx];
      cand.marked = old.marked;
      cand.dirty = !sameGeometry(old.ring, cand.ring);   // 几何没变则不重画
      consumed[idx] = true;
    }
    next.push_back(std::move(cand));
  }
  // 未被本帧重投的旧表面保留(等 TTL 过期), 其 marked 不丢失
  for (size_t i = 0; i < surfaces_.size(); ++i) {
    if (!consumed[i]) {next.push_back(std::move(surfaces_[i]));}
  }
  surfaces_ = std::move(next);
}

namespace
{
bool ringCentroid(const std::vector<geometry_msgs::msg::Point> & r, double & x, double & y)
{
  if (r.empty()) {return false;}
  x = 0; y = 0;
  for (const auto & p : r) {x += p.x; y += p.y;}
  x /= r.size(); y /= r.size();
  return true;
}
}  // namespace

bool SurfaceOverrideLayer::sameGeometry(
  const std::vector<geometry_msgs::msg::Point> & a,
  const std::vector<geometry_msgs::msg::Point> & b)
{
  if (a.size() != b.size()) {return false;}
  double ax, ay, bx, by;
  if (!ringCentroid(a, ax, ay) || !ringCentroid(b, bx, by)) {return false;}
  return std::hypot(ax - bx, ay - by) < 0.05;
}

int SurfaceOverrideLayer::matchIndex(const Surface & cand) const
{
  double cx = 0, cy = 0;
  for (const auto & p : cand.ring) {cx += p.x; cy += p.y;}
  if (!cand.ring.empty()) {cx /= cand.ring.size(); cy /= cand.ring.size();}
  for (size_t i = 0; i < surfaces_.size(); ++i) {
    const auto & old = surfaces_[i];
    if (old.class_name != cand.class_name || old.ring.empty()) {continue;}
    double ox = 0, oy = 0;
    for (const auto & p : old.ring) {ox += p.x; oy += p.y;}
    ox /= old.ring.size(); oy /= old.ring.size();
    if (std::hypot(ox - cx, oy - cy) < merge_dist_) {return static_cast<int>(i);}
  }
  return -1;
}

void SurfaceOverrideLayer::restore(Surface & s, Costmap2D * master)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*master->getMutex());
  for (auto & [wx, wy, prev] : s.marked) {
    unsigned int mx, my;
    if (!master->worldToMap(wx, wy, mx, my)) {continue;}
    const unsigned int idx = master->getIndex(mx, my);
    // 只在我们写的代价仍然生效时恢复（不覆盖别人写的更高代价）
    if (master->getCost(idx) <= s.cost) {
      master->setCost(mx, my, prev);
    }
  }
  s.marked.clear();
}

void SurfaceOverrideLayer::expire(const rclcpp::Time & now, Costmap2D * master)
{
  auto it = surfaces_.begin();
  while (it != surfaces_.end()) {
    const double age = (now - it->stamp).seconds();
    if (it->ttl > 0.0 && age > it->ttl) {
      restore(*it, master);
      it = surfaces_.erase(it);
    } else {
      ++it;
    }
  }
}

void SurfaceOverrideLayer::updateBounds(
  double, double, double, double * min_x, double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<std::mutex> lk(mu_);
  auto * master = layered_costmap_->getCostmap();
  const rclcpp::Time now = clock_->now();
  if (master) {
    expire(now, master);
  }
  for (const auto & s : surfaces_) {
    for (const auto & p : s.ring) {
      *min_x = std::min(*min_x, p.x);
      *min_y = std::min(*min_y, p.y);
      *max_x = std::max(*max_x, p.x);
      *max_y = std::max(*max_y, p.y);
    }
  }
}

void SurfaceOverrideLayer::updateCosts(Costmap2D & master_grid, int min_i, int min_j,
  int max_i, int max_j)
{
  std::lock_guard<std::mutex> lk(mu_);
  const rclcpp::Time now = clock_->now();
  for (auto & s : surfaces_) {
    if (!s.dirty && (now - s.last_draw).seconds() < refresh_period_) {
      continue;                              // 几何未变: 不重画(消除发布闪烁)
    }
    // 先恢复上一轮写的格子再重画，保证几何移动不留残影
    restore(s, &master_grid);
    s.dirty = false;
    s.last_draw = now;
    double x0 = 1e9, y0 = 1e9, x1 = -1e9, y1 = -1e9;
    for (const auto & p : s.ring) {
      x0 = std::min(x0, p.x); y0 = std::min(y0, p.y);
      x1 = std::max(x1, p.x); y1 = std::max(y1, p.y);
    }
    unsigned int mx0, my0, mx1, my1;
    if (!master_grid.worldToMap(x0, y0, mx0, my0) ||
      !master_grid.worldToMap(x1, y1, mx1, my1))
    {
      continue;
    }
    const int ix0 = std::max(static_cast<int>(mx0) - 1, min_i);
    const int ix1 = std::min(static_cast<int>(mx1) + 1, max_i - 1);
    const int iy0 = std::max(static_cast<int>(my0) - 1, min_j);
    const int iy1 = std::min(static_cast<int>(my1) + 1, max_j - 1);
    std::lock_guard<Costmap2D::mutex_t> guard(*master_grid.getMutex());
    for (int iy = iy0; iy <= iy1; ++iy) {
      for (int ix = ix0; ix <= ix1; ++ix) {
        double wx, wy;
        master_grid.mapToWorld(ix, iy, wx, wy);
        if (!pointInRing(s.ring, wx, wy)) {continue;}
        const unsigned int idx = master_grid.getIndex(ix, iy);
        const unsigned char cur = master_grid.getCost(idx);
        if (cur < s.cost) {
          s.marked.emplace_back(wx, wy, cur);
          master_grid.setCost(ix, iy, s.cost);
        }
      }
    }
  }
  current_ = true;
}

void SurfaceOverrideLayer::reset()
{
  std::lock_guard<std::mutex> lk(mu_);
  auto * master = layered_costmap_->getCostmap();
  if (master) {
    for (auto & s : surfaces_) {
      restore(s, master);
    }
  }
  surfaces_.clear();
}

void SurfaceOverrideLayer::deactivate()
{
  sub_.reset();
}

void SurfaceOverrideLayer::activate()
{
  auto node = node_.lock();
  if (!node) {return;}
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  sub_ = node->create_subscription<fp_surface_guard::msg::SurfaceArray>(
    topic_, rclcpp::QoS(10),
    [this](fp_surface_guard::msg::SurfaceArray::SharedPtr msg) {cb(msg);},
    options);
}

}  // namespace fp_surface_guard

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(fp_surface_guard::SurfaceOverrideLayer, nav2_costmap_2d::Layer)
