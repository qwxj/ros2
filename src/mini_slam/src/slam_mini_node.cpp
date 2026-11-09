#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <std_srvs/srv/trigger.hpp>

#include "mini_slam/pose2d.hpp"
#include "mini_slam/map_io.hpp"
#include "mini_slam/occupancy_grid.hpp"
#include "mini_slam/pose_graph.hpp"
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

    // ---------- 位姿图（后端）----------
    //   前端（扫描匹配）负责"每帧准"，后端（位姿图）负责"全局一致"。
    //   关键帧：每移动 keyframe_dist 米 或 转过 keyframe_angle 弧度，就记一个节点。
    use_pose_graph_ = declare_parameter<bool>("use_pose_graph", true);
    keyframe_dist_ = declare_parameter<double>("keyframe_dist", 0.30);
    keyframe_angle_ = declare_parameter<double>("keyframe_angle", 0.15);
    loop_radius_ = declare_parameter<double>("loop_radius", 2.0);
    loop_min_gap_ = declare_parameter<int>("loop_min_gap", 10);
    loop_confirm_score_ = declare_parameter<double>("loop_confirm_score", 0.60);
    optimize_iterations_ = declare_parameter<int>("optimize_iterations", 50);

    // ---------- 回环后重建地图 ----------
    //   方案 A 只修轨迹、不重建地图 → 墙上仍重影。
    //   这里每帧记下「属于哪个关键帧 + 相对该关键帧的位姿 + 激光」，
    //   优化后按 (优化后的关键帧位姿 ⊕ 相对位姿) 重算每帧位置，
    //   然后清空地图、把所有光线重放一遍。
    use_map_rebuild_ = declare_parameter<bool>("use_map_rebuild", true);
    map_rebuild_max_frames_ = declare_parameter<int>("map_rebuild_max_frames", 3000);

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

      save_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/save_map",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std_srvs::srv::Trigger::Response::SharedPtr res)
      {
        std::string msg;
        res->success = saveToDisk(msg);
        res->message = msg;
        if (res->success) {
          RCLCPP_INFO(get_logger(), "save_map: %s", msg.c_str());
        } else {
          RCLCPP_WARN(get_logger(), "save_map 失败: %s", msg.c_str());
        }
      });

    }

    last_publish_ = now().seconds();

    RCLCPP_INFO(get_logger(),
      "slam_mini 启动：%.0f m x %.0f m @ %.3f m/px (%dx%d)，搜索 ±%.2f m / ±%.2f rad",
      map_size_m_, map_size_m_, resolution_, n, n, search_xy_, search_yaw_);
  }

  ~SlamMiniNode() override
  {
    if (!has_first_scan_ || map_path_.empty()) {return;}
    std::string msg;
    if (saveToDisk(msg)) {
      RCLCPP_INFO(get_logger(), "%s", msg.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "%s", msg.c_str());
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

    // ---------- 位姿图：关键帧 + 回环 + 优化 ----------
    //   注意顺序：必须在 raycast 之前 —— 优化会修正 pose_，
    //   建图要用修正后的位姿，否则地图还是歪的。
    if (use_pose_graph_) {
      stepPoseGraph(msg, inc, ranges);
    }

    raycastIntoMap(msg);

    // 存档这一帧，供回环后重建地图用
    if (use_map_rebuild_ && last_keyframe_id_ >= 0) {
      captureFrame(msg, inc, ranges);
    }

    if (publish_tf_) {publishTf(msg->header.stamp);}
    if (t - last_publish_ >= publish_period_) {publishMap(t);}
  }

  /// a ⊕ b：把 b 从 a 的坐标系变换到世界系
  static Pose2D compose(const Pose2D & a, const Pose2D & b)
  {
    const double c = std::cos(a.yaw);
    const double s = std::sin(a.yaw);
    Pose2D r;
    r.x = a.x + c * b.x - s * b.y;
    r.y = a.y + s * b.x + c * b.y;
    r.yaw = wrapToPi(a.yaw + b.yaw);
    return r;
  }

  /// 存档当前帧（相对当前关键帧的位姿 + 激光）
  void captureFrame(const sensor_msgs::msg::LaserScan::SharedPtr msg,
                    double angle_inc,
                    const std::vector<float> & ranges)
  {
    // 帧数上限：超了就隔一帧丢一帧（保留覆盖，限制内存）
    if (static_cast<int>(frame_records_.size()) >= map_rebuild_max_frames_) {
      std::vector<FrameRecord> kept;
      kept.reserve(frame_records_.size() / 2 + 1);
      for (size_t i = 0; i < frame_records_.size(); i += 2) {
        kept.push_back(std::move(frame_records_[i]));
      }
      frame_records_.swap(kept);
    }

    FrameRecord f;
    f.kf_id = last_keyframe_id_;
    double rx = 0.0, ry = 0.0, ryaw = 0.0;
    relativePoseOut(graph_.pose(last_keyframe_id_), pose_, rx, ry, ryaw);
    f.rel.x = rx; f.rel.y = ry; f.rel.yaw = ryaw;
    f.ranges = ranges;
    f.angle_min = msg->angle_min;
    f.angle_increment = angle_inc;
    f.range_max = msg->range_max;
    frame_records_.push_back(std::move(f));
  }

  /// 用优化后的关键帧位姿重算每帧位置，清空地图后重放所有光线
  void rebuildFromFrames()
  {
    if (frame_records_.empty()) {return;}

    // 重建一张同样尺寸的空地图
    const int n = std::max(1, static_cast<int>(std::lround(map_size_m_ / resolution_)));
    map_ = std::make_unique<OccupancyGrid>(
      resolution_, -map_size_m_ * 0.5, -map_size_m_ * 0.5, n, n);

    for (const auto & f : frame_records_) {
      if (f.kf_id < 0 || f.kf_id >= graph_.size()) {continue;}
      // ★ 关键：帧位姿 = 优化后的关键帧位姿 ⊕ 捕获时的相对位姿
      const Pose2D p = compose(graph_.pose(f.kf_id), f.rel);

      for (size_t i = 0; i < f.ranges.size(); ++i) {
        const double r = f.ranges[i];
        if (!std::isfinite(r) || r < 0.05 || r >= f.range_max) {continue;}
        const double a = p.yaw + f.angle_min + static_cast<double>(i) * f.angle_increment;
        map_->raycast(p.x, p.y,
                      p.x + r * std::cos(a), p.y + r * std::sin(a));
      }
    }
    ++map_rebuild_count_;

    // 统计重建后的占据格数，方便对比"重影有没有减少"
    int occupied = 0;
    for (int gy = 0; gy < map_->height(); ++gy) {
      for (int gx = 0; gx < map_->width(); ++gx) {
        if (map_->probability(gx, gy) > 50) {++occupied;}
      }
    }
    RCLCPP_INFO(get_logger(),
                "地图已用 %zu 帧重建（第 %d 次），占据格 %d 个",
                frame_records_.size(), map_rebuild_count_, occupied);
  }

  /// a -> b 的相对位姿（在 a 的坐标系下），也就是 inv(a) * b
  static void relativePoseOut(const Pose2D & a, const Pose2D & b,
                              double & dx, double & dy, double & dyaw)
  {
    const double c = std::cos(a.yaw);
    const double s = std::sin(a.yaw);
    const double ex = b.x - a.x;
    const double ey = b.y - a.y;
    dx =  c * ex + s * ey;
    dy = -s * ex + c * ey;
    dyaw = wrapToPi(b.yaw - a.yaw);
  }

  /// 位姿图的一步：判断要不要加关键帧；加了之后试回环；有回环就优化。
  void stepPoseGraph(const sensor_msgs::msg::LaserScan::SharedPtr msg,
                     double angle_inc,
                     const std::vector<float> & ranges)
  {
    // ---- 第一个关键帧 ----
    if (last_keyframe_id_ < 0) {
      last_keyframe_id_ = graph_.addNode(pose_.x, pose_.y, pose_.yaw);
      kf_odom_.push_back(odom_pose_);      // ★ 记下这个关键帧时刻的 odom 位姿
      RCLCPP_INFO(get_logger(), "位姿图：记下第 1 个关键帧 (%.2f, %.2f, %.2f)",
                  pose_.x, pose_.y, pose_.yaw);
      return;
    }

    // ---- 够远/够转才加新关键帧 ----
    const Pose2D prev = graph_.pose(last_keyframe_id_);
    const double d = std::hypot(pose_.x - prev.x, pose_.y - prev.y);
    const double da = std::fabs(wrapToPi(pose_.yaw - prev.yaw));
    if (d < keyframe_dist_ && da < keyframe_angle_) {return;}

    // odom 边的观测 = 【扫描匹配后的相对位姿】。
    //
    //   为什么不直接用原始 odom 相对位姿（试过，会发散）：
    //     节点位姿是扫描匹配的结果，已经纠正过 odom 漂移；
    //     而原始 odom 边是未纠正的。两者天生不一致 →
    //     优化器为了满足 odom 边，把节点位姿【拉回 odom 积分】→
    //     等于撤销扫描匹配，下一帧拿着一张和位姿不一致的地图去匹配 → 越跑越歪。
    //
    //   正确做法：odom 只用来做"初始猜测"（在 onScan 开头），
    //   图里的相邻约束用【精化后】的相对位姿；真正带独立信息的是回环边。
    const int cur = graph_.addNode(pose_.x, pose_.y, pose_.yaw);
    kf_odom_.push_back(odom_pose_);        // 仅留作调试，不参与建边

    double dx = 0.0, dy = 0.0, dyaw = 0.0;
    relativePoseOut(prev, pose_, dx, dy, dyaw);
    graph_.addEdge(last_keyframe_id_, cur, dx, dy, dyaw, 1.0, 1.0, false);

    RCLCPP_INFO(get_logger(), "位姿图：加关键帧 #%d (%.2f, %.2f, %.2f)，%d 个节点",
                cur, pose_.x, pose_.y, pose_.yaw, graph_.size());

    const int prev_kf = last_keyframe_id_;
    last_keyframe_id_ = cur;

    // ---- 回环检测 + 验证 ----
    if (graph_.size() < loop_min_gap_ + 2) {return;}

    const auto cands = graph_.findLoopCandidates(cur, loop_radius_, loop_min_gap_);
    if (cands.empty()) {return;}

    // 用扫描匹配在当前地图上确认候选：得分够高才认为是真回环。
    //   只靠空间距离不行 —— 相似走廊会误判，把地图撕裂。
    //   一帧可能有多个候选都在半径内（相邻关键帧离得很近），
    //   所以只取【得分最高】的那一个，否则同一帧会加好几条重复边。
    int best_id = -1;
    double best_score = 0.0;
    ScanMatchResult best{};

    for (int c : cands) {
      const Pose2D p = graph_.pose(c);
      const auto r = sm_.match(
        ranges, msg->angle_min, angle_inc, msg->range_max,
        p.x, p.y, p.yaw, 0.30, 0.30, 7);

      if (r.score < loop_confirm_score_) {continue;}
      if (r.score > best_score) {
        best_score = r.score;
        best_id = c;
        best = r;
      }
    }

    if (best_id < 0) {return;}

    // ★ 回环边的观测 = 【扫描匹配算出来的位姿】（独立于当前位姿估计），
    //   不是从两个节点位姿反算，否则又变成"自洽"、误差恒 0。
    {
      const Pose2D matched{best.x, best.y, best.yaw};
      double lx = 0.0, ly = 0.0, lyaw = 0.0;
      relativePoseOut(graph_.pose(best_id), matched, lx, ly, lyaw);

      graph_.addEdge(best_id, cur, lx, ly, lyaw, 2.0, 2.0, true);
      ++loop_count_;
      RCLCPP_INFO(get_logger(),
                  "★ 回环闭合 #%d -> #%d，得分 %.3f，匹配位姿 (%.2f, %.2f, %.2f)（第 %d 次）",
                  best_id, cur, best_score, best.x, best.y, best.yaw, loop_count_);
    }

    if (graph_.edgeCount() == 0) {return;}

    // ---- 优化，并把当前位姿拉到优化后的位置 ----
    const double err_before = graph_.totalError();
    const double err_after = graph_.optimize(optimize_iterations_, 0.5);
    const Pose2D opt = graph_.pose(cur);
    const double shift = std::hypot(opt.x - pose_.x, opt.y - pose_.y);

    // 把整个位姿往优化值上拉。只改 pose_，不改地图（方案 A：只修轨迹）
    pose_.x = opt.x;
    pose_.y = opt.y;
    pose_.yaw = opt.yaw;

    RCLCPP_INFO(get_logger(),
                "位姿图优化：总误差 %.4f -> %.4f，当前节点挪了 %.3f m",
                err_before, err_after, shift);

    // ★ 方案 B：用优化后的位姿把所有帧的地图重算一遍
    if (use_map_rebuild_) {
      rebuildFromFrames();
    }
    (void)prev_kf;
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


  bool saveToDisk(std::string & msg)
  {
    if (!has_first_scan_) {
      msg = "还没有收到第一帧扫描，地图是空的";
      return false;
    }
    if (map_path_.empty()) {
      msg = "参数 map_path 为空，不知道存到哪";
      return false;
    }

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
    if (!saveMap(yaml, *map_, meta, &err)) {
      msg = "保存失败: " + err;
      return false;
    }

    char buf[256];
    std::snprintf(buf, sizeof(buf), "已保存 %s（%d x %d，%d 帧）",
                  yaml.c_str(), map_->width(), map_->height(), frames_);
    msg = buf;
    return true;
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

  // 位姿图（后端）
  PoseGraph graph_;
  bool use_pose_graph_{true};
  double keyframe_dist_{0.30};
  double keyframe_angle_{0.15};
  double loop_radius_{2.0};
  int loop_min_gap_{10};
  double loop_confirm_score_{0.60};
  int optimize_iterations_{50};
  /// 一帧的存档：属于哪个关键帧、相对该关键帧的位姿、以及激光数据。
  ///   存「相对位姿」而不是绝对位姿，是为了回环优化后能跟着关键帧一起移动。
  struct FrameRecord
  {
    int kf_id{-1};
    Pose2D rel;                              // 在关键帧坐标系下的位姿
    std::vector<float> ranges;
    double angle_min{0.0};
    double angle_increment{0.0};
    double range_max{0.0};
  };
  std::vector<FrameRecord> frame_records_;
  bool use_map_rebuild_{true};
  int map_rebuild_max_frames_{3000};
  int map_rebuild_count_{0};

  std::vector<Pose2D> kf_odom_;   // 每个关键帧时刻的 odom 位姿（odom 边的观测来源）
  int last_keyframe_id_{-1};
  int loop_count_{0};
  Pose2D pose_, odom_pose_, odom_prev_, odom_at_start_;
  bool has_first_scan_{false}, has_odom_{false};
  int frames_{0}, matched_{0}, rejected_{0};
  double last_publish_{0.0};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_srv_;
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
