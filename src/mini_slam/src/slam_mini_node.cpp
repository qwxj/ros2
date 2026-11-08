#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "mini_slam/pose2d.hpp"
#include "mini_slam/map_io.hpp"
#include "mini_slam/occupancy_grid.hpp"
#include "mini_slam/scan_matcher.hpp"

namespace mini_slam
{

class SlamMiniNode : public rclcpp::Node
{
public:
  SlamMiniNode() : Node("slam_mini")
  {
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);

    resolution_ = declare_parameter<double>("resolution", 0.05);
    map_size_m_ = declare_parameter<double>("map_size_m", 12.0);
    search_xy_ = declare_parameter<double>("search_xy", 0.15);
    search_yaw_ = declare_parameter<double>("search_yaw", 0.15);
    search_steps_ = declare_parameter<int>("search_steps", 7);
    min_score_ = declare_parameter<double>("min_score", 0.35);
    publish_period_ = declare_parameter<double>("publish_period", 1.0);
    map_path_ = declare_parameter<std::string>("map_path", "/tmp/mini_slam_map");

    const int n = std::max(1, static_cast<int>(std::lround(map_size_m_ / resolution_)));
    map_ = std::make_unique<OccupancyGrid>(
      resolution_, -map_size_m_ * 0.5, -map_size_m_ * 0.5, n, n);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr m) {onScan(m);});

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr m) {onOdom(m);});

    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).transient_local().reliable());

    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    last_publish_ = now().seconds();

    RCLCPP_INFO(get_logger(),
      "slam_mini 启动：%.0f m x %.0f m @ %.3f m/px (%dx%d)，搜索 ±%.2f m / ±%.2f rad",
      map_size_m_, map_size_m_, resolution_, n, n, search_xy_, search_yaw_);
  }

  ~SlamMiniNode() override
  {
    if (!has_first_scan_ || map_path_.empty()) {return;}
    MapMetadata meta;
    // ★ 只写【文件名】，不写绝对路径。
    //   标准 ROS 地图的 image 字段是相对 yaml 所在目录的，
    //   这样 yaml + pgm 可以一起拷到任何机器 / 进 git 仓库。
    const size_t slash = map_path_.find_last_of('/');
    meta.image = (slash == std::string::npos)
      ? (map_path_ + ".pgm")
      : (map_path_.substr(slash + 1) + ".pgm");
    meta.resolution = map_->resolution();
    meta.origin_x = map_->originX();
    meta.origin_y = map_->originY();
    meta.negate = 0;
    meta.occupied_thresh = 0.65;
    meta.free_thresh = 0.15;

    std::string err;
    const std::string yaml = map_path_ + ".yaml";
    if (saveMap(yaml, *map_, meta, &err)) {
      RCLCPP_INFO(get_logger(), "地图已保存到 %s", yaml.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "保存地图失败: %s", err.c_str());
    }
  }

private:
  static Pose2D poseFromMsg(const nav_msgs::msg::Odometry & m)
  {
    Pose2D p;
    p.x = m.pose.pose.position.x;
    p.y = m.pose.pose.position.y;
    const auto & q = m.pose.pose.orientation;
    p.yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                       1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    return p;
  }

  static geometry_msgs::msg::Quaternion quatFromYaw(double yaw)
  {
    geometry_msgs::msg::Quaternion q;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_pose_ = poseFromMsg(*msg);
    has_odom_ = true;
  }

  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const double t = now().seconds();

    if (!has_first_scan_) {
      // 第一帧：约定 SLAM 起点为 (0,0,0)，也就是 map == odom 的初始对齐
      pose_.x = 0.0; pose_.y = 0.0; pose_.yaw = 0.0;
      if (has_odom_) {
        odom_at_start_ = odom_pose_;
        // 把起点对齐到 odom 的当前位置
        pose_.x = odom_pose_.x;
        pose_.y = odom_pose_.y;
        pose_.yaw = odom_pose_.yaw;
      }
      raycastIntoMap(msg);
      has_first_scan_ = true;
      publishMap(t);
      RCLCPP_INFO(get_logger(), "收到第一帧扫描，开始建图");
      return;
    }

    // ---------- 1. 用 odom 增量做猜测 ----------
    double gx = pose_.x, gy = pose_.y, gyaw = pose_.yaw;
    if (has_odom_) {
      const double dx = odom_pose_.x - odom_prev_.x;
      const double dy = odom_pose_.y - odom_prev_.y;
      const double dyaw = wrapToPi(odom_pose_.yaw - odom_prev_.yaw);
      const double c = std::cos(pose_.yaw), s = std::sin(pose_.yaw);
      gx += c * dx - s * dy;
      gy += s * dx + c * dy;
      gyaw = wrapToPi(pose_.yaw + dyaw);
      odom_prev_ = odom_pose_;
    }

    // ---------- 2. 扫描匹配 ----------
    std::vector<float> ranges;
    const size_t n = msg->ranges.size();
    const int step = std::max(1, static_cast<int>(n) / 180);
    for (size_t i = 0; i < n; i += static_cast<size_t>(step)) {
      ranges.push_back(msg->ranges[i]);
    }
    const double inc = msg->angle_increment * step;

    sm_.setMap(*map_);
    const auto r = sm_.match(ranges, msg->angle_min, inc, msg->range_max,
                             gx, gy, gyaw, search_xy_, search_yaw_, search_steps_);

    if (r.score >= min_score_) {
      pose_.x = r.x; pose_.y = r.y; pose_.yaw = r.yaw;
      ++matched_;
    } else {
      pose_.x = gx; pose_.y = gy; pose_.yaw = gyaw;
      ++rejected_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "匹配得分过低 %.3f < %.3f，退回 odom 猜测", r.score, min_score_);
    }
    ++frames_;

    raycastIntoMap(msg);

    if (publish_tf_) {publishTf(msg->header.stamp);}
    if (t - last_publish_ >= publish_period_) {publishMap(t);}
  }

  void raycastIntoMap(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const double r = msg->ranges[i];
      if (!std::isfinite(r) || r < msg->range_min || r >= msg->range_max) {continue;}
      const double a = pose_.yaw + msg->angle_min +
        static_cast<double>(i) * msg->angle_increment;
      map_->raycast(pose_.x, pose_.y,
                    pose_.x + r * std::cos(a), pose_.y + r * std::sin(a));
    }
  }

  void publishMap(double t)
  {
    last_publish_ = t;
    nav_msgs::msg::OccupancyGrid out;
    out.header.stamp = now();
    out.header.frame_id = map_frame_;
    out.info.resolution = map_->resolution();
    out.info.width = static_cast<uint32_t>(map_->width());
    out.info.height = static_cast<uint32_t>(map_->height());
    out.info.origin.position.x = map_->originX();
    out.info.origin.position.y = map_->originY();
    out.info.origin.orientation.w = 1.0;
    out.data = map_->toOccupancyData();
    map_pub_->publish(out);

    RCLCPP_INFO(get_logger(),
      "帧 %d：位姿 (%.2f, %.2f, %.2f)  匹配 %d / 退回 %d",
      frames_, pose_.x, pose_.y, pose_.yaw, matched_, rejected_);
  }

  void publishTf(const rclcpp::Time & stamp)
  {
    // map->odom = (map->base) * inv(odom->base)
    const double c = std::cos(pose_.yaw), s = std::sin(pose_.yaw);
    const double co = std::cos(odom_pose_.yaw), so = std::sin(odom_pose_.yaw);
    const double ix = -co * odom_pose_.x - so * odom_pose_.y;
    const double iy = so * odom_pose_.x - co * odom_pose_.y;

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = map_frame_;
    tf.child_frame_id = odom_frame_;
    tf.transform.translation.x = pose_.x + c * ix - s * iy;
    tf.transform.translation.y = pose_.y + s * ix + c * iy;
    tf.transform.rotation = quatFromYaw(wrapToPi(pose_.yaw - odom_pose_.yaw));
    tf_broadcaster_->sendTransform(tf);
  }

  std::string scan_topic_, odom_topic_, map_frame_, odom_frame_, base_frame_, map_path_;
  bool publish_tf_{true};
  double resolution_{0.05}, map_size_m_{12.0};
  double search_xy_{0.15}, search_yaw_{0.15};
  int search_steps_{7};
  double min_score_{0.35}, publish_period_{1.0};

  std::unique_ptr<OccupancyGrid> map_;
  ScanMatcher sm_;
  Pose2D pose_, odom_pose_, odom_prev_, odom_at_start_;
  bool has_first_scan_{false}, has_odom_{false};
  int frames_{0}, matched_{0}, rejected_{0};
  double last_publish_{0.0};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace mini_slam

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mini_slam::SlamMiniNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  node.reset();
  return 0;
}
