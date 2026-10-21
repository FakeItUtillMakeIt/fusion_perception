// Patchwork++ 地面分割节点：与 z 带法 pointcloud_preprocess 输出接口一致的并行方案。
// 输入原始点云(sensor 系) -> PatchWorkpp 分地面/非地面 -> TF 到 base_link -> 距离/高度过滤发布。
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <cstring>

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "patchwork/patchworkpp.h"

using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

namespace {

struct ParsedCloud {
  std::vector<Eigen::Vector3f> pts;
  std::vector<float> inten;
  bool has_int{false};
};

ParsedCloud parse_cloud(const PointCloud2 &m) {
  std::unordered_map<std::string, uint32_t> off;
  for (const auto &f : m.fields) off[f.name] = f.offset;
  ParsedCloud out;
  if (!off.count("x") || !off.count("y") || !off.count("z")) return out;
  out.has_int = off.count("intensity") > 0;
  out.pts.reserve(m.width * m.height);
  for (uint32_t i = 0; i < m.width * m.height; ++i) {
    const uint8_t *p = m.data.data() + size_t(i) * m.point_step;
    Eigen::Vector3f v(*reinterpret_cast<const float *>(p + off["x"]),
                      *reinterpret_cast<const float *>(p + off["y"]),
                      *reinterpret_cast<const float *>(p + off["z"]));
    if (!std::isfinite(v.x()) || !std::isfinite(v.y()) || !std::isfinite(v.z())) continue;
    out.pts.push_back(v);
    if (out.has_int) {
      uint16_t raw;
      std::memcpy(&raw, p + off["intensity"], 2);
      out.inten.push_back(static_cast<float>(raw));
    }
  }
  return out;
}

PointCloud2 make_cloud(const std_msgs::msg::Header &hdr, const std::vector<Eigen::Vector3f> &pts) {
  PointCloud2 m;
  m.header = hdr;
  m.height = 1;
  m.width = pts.size();
  m.fields = {[] (std::string n, uint32_t o) { PointField f; f.name = n; f.offset = o;
      f.datatype = PointField::FLOAT32; f.count = 1; return f; }("x", 0),
      [] (std::string n, uint32_t o) { PointField f; f.name = n; f.offset = o;
      f.datatype = PointField::FLOAT32; f.count = 1; return f; }("y", 4),
      [] (std::string n, uint32_t o) { PointField f; f.name = n; f.offset = o;
      f.datatype = PointField::FLOAT32; f.count = 1; return f; }("z", 8)};
  m.is_bigendian = false;
  m.point_step = 12;
  m.row_step = 12 * m.width;
  m.is_dense = true;
  m.data.resize(12 * pts.size());
  for (size_t i = 0; i < pts.size(); ++i)
    std::memcpy(m.data.data() + 12 * i, pts[i].data(), 12);
  return m;
}

Eigen::Matrix3f quat_to_rot(double x, double y, double z, double w) {
  Eigen::Matrix3f R;
  R << 1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
       2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
       2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y);
  return R;
}

}  // namespace

class PatchworkGroundNode : public rclcpp::Node {
 public:
  PatchworkGroundNode() : Node("patchwork_ground") {
    in_topic_ = declare_parameter<std::string>("input_topic", "/lidar/points_raw");
    ng_topic_ = declare_parameter<std::string>("ng_topic", "/fp_ob/points_non_ground_pw");
    g_topic_ = declare_parameter<std::string>("g_topic", "/fp_ob/points_ground_pw");
    tf_parent_ = declare_parameter<std::string>("tf_parent", "base_link");
    sensor_height_ = declare_parameter<double>("sensor_height", 0.525);
    min_range_ = declare_parameter<double>("min_range", 2.0);
    max_range_ = declare_parameter<double>("max_range", 20.0);
    max_z_ = declare_parameter<double>("max_z", 2.5);
    th_seeds_ = declare_parameter<double>("th_seeds", 0.05);
    th_dist_ = declare_parameter<double>("th_dist", 0.05);
    th_seeds_v_ = declare_parameter<double>("th_seeds_v", 0.1);
    th_dist_v_ = declare_parameter<double>("th_dist_v", 0.05);
    uprightness_ = declare_parameter<double>("uprightness_thr", 0.707);
    num_min_pts_ = declare_parameter<int>("num_min_pts", 5);

    patchwork::Params params;
    params.verbose = false;
    params.enable_TGR = declare_parameter<bool>("enable_TGR", true);
    params.enable_RNR = declare_parameter<bool>("enable_RNR", false);
    params.enable_RVPF = declare_parameter<bool>("enable_RVPF", true);
    params.sensor_height = sensor_height_;
    params.min_range = min_range_;
    params.max_range = max_range_;
    params.th_seeds = th_seeds_;
    params.th_dist = th_dist_;
    params.th_seeds_v = th_seeds_v_;
    params.th_dist_v = th_dist_v_;
    params.uprightness_thr = uprightness_;
    params.num_min_pts = num_min_pts_;
    pw_ = std::make_unique<patchwork::PatchWorkpp>(params);

    buf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tfl_ = std::make_shared<tf2_ros::TransformListener>(*buf_);
    pub_ng_ = create_publisher<PointCloud2>(ng_topic_, 10);
    pub_g_ = create_publisher<PointCloud2>(g_topic_, 10);
    sub_ = create_subscription<PointCloud2>(
        in_topic_, rclcpp::SensorDataQoS(),
        std::bind(&PatchworkGroundNode::cb, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "patchwork_ground up: %s -> %s / %s (h=%.3f)",
                in_topic_.c_str(), ng_topic_.c_str(), g_topic_.c_str(), sensor_height_);
  }

 private:
  void cb(const PointCloud2::SharedPtr msg) {
    auto parsed = parse_cloud(*msg);
    auto &pts = parsed.pts;
    if (pts.size() < 50) return;
    Eigen::MatrixXf cloud(pts.size(), parsed.has_int ? 4 : 3);
    for (size_t i = 0; i < pts.size(); ++i) {
      cloud.row(i).head<3>() = pts[i];
      if (parsed.has_int) cloud(i, 3) = parsed.inten[i];
    }

    auto t0 = now();
    pw_->estimateGround(cloud);
    Eigen::VectorXi gi = pw_->getGroundIndices();
    Eigen::VectorXi ni = pw_->getNongroundIndices();

    geometry_msgs::msg::TransformStamped tfm;
    try {
      tfm = buf_->lookupTransform(tf_parent_, msg->header.frame_id, msg->header.stamp);
    } catch (const std::exception &) {
      try {
        tfm = buf_->lookupTransform(tf_parent_, msg->header.frame_id, tf2::TimePointZero);
      } catch (const std::exception &) {
        return;
      }
    }
    Eigen::Matrix3f R = quat_to_rot(tfm.transform.rotation.x, tfm.transform.rotation.y,
                                    tfm.transform.rotation.z, tfm.transform.rotation.w);
    Eigen::Vector3f t(tfm.transform.translation.x, tfm.transform.translation.y,
                      tfm.transform.translation.z);

    std::vector<Eigen::Vector3f> ng, g;
    ng.reserve(ni.size());
    g.reserve(gi.size());
    auto take = [&](const Eigen::VectorXi &idx, std::vector<Eigen::Vector3f> &out) {
      for (int k = 0; k < idx.size(); ++k) {
        const Eigen::Vector3f &p = pts[idx(k)];
        Eigen::Vector3f b = R * p + t;
        float r = std::hypot(b.x(), b.y());
        if (r < min_range_ || r > max_range_ || b.z() > max_z_) continue;
        out.push_back(b);
      }
    };
    take(ni, ng);
    take(gi, g);

    std_msgs::msg::Header hdr;
    hdr.stamp = msg->header.stamp;
    hdr.frame_id = tf_parent_;
    pub_ng_->publish(make_cloud(hdr, ng));
    pub_g_->publish(make_cloud(hdr, g));
    if (first_) {
      RCLCPP_INFO(get_logger(), "first out: ng=%zu g=%zu (%.1f ms)", ng.size(), g.size(),
                  (now() - t0).seconds() * 1e3);
      first_ = false;
    }
  }

  std::string in_topic_, ng_topic_, g_topic_, tf_parent_;
  double sensor_height_, min_range_, max_range_, max_z_;
  double th_seeds_, th_dist_, th_seeds_v_, th_dist_v_, uprightness_;
  int num_min_pts_;
  bool first_{true};
  std::unique_ptr<patchwork::PatchWorkpp> pw_;
  std::shared_ptr<tf2_ros::Buffer> buf_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_ng_, pub_g_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PatchworkGroundNode>());
  rclcpp::shutdown();
  return 0;
}
