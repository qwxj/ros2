#include <memory>
#include <random>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "mini_slam/pose2d.hpp"
#include "mini_slam/particle_filter.hpp"

namespace mini_slam
{

class AmclMiniNode : public rclcpp::Node
{
public:
  AmclMiniNode() : Node("amcl_mini"), rng_(std::random_device{}())
  {
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    map_topic_ = declare_parameter<std::string>("map_topic", "/map");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);

    const int max_particles = declare_parameter<int>("max_particles", 2000);
    const int min_particles = declare_parameter<int>("min_particles", 300);
    const double kld_err = declare_parameter<double>("kld_err", 0.05);
    const double kld_z = declare_parameter<double>("kld_z", 0.99);
    alpha_slow_ = declare_parameter<double>("alpha_slow", 0.001);
    alpha_fast_ = declare_parameter<double>("alpha_fast", 0.1);

    // ★ 运动模型噪声 alpha1~4（odom 差分模型）
    //   这四个决定了"有多不信任里程计"，不同底盘差一个数量级：
    //     差速轮（干净地面）  0.05 / 0.01 / 0.05 / 0.01   ← 默认
    //     差速轮（打滑严重）  0.20 / 0.05 / 0.20 / 0.05
    //     履带                0.30 / 0.10 / 0.30 / 0.10
    //     全向轮              rot 小、trans 大：0.02 / 0.05 / 0.05 / 0.10
    //   var_rot1  = alpha1*rot1^2 + alpha2*trans^2
    //   var_trans = alpha3*trans^2 + alpha4*rot1^2 + alpha4*rot2^2
    //   var_rot2  = alpha1*rot2^2 + alpha2*trans^2
    alpha1_ = declare_parameter<double>("alpha1", 0.05);
    alpha2_ = declare_parameter<double>("alpha2", 0.01);
    alpha3_ = declare_parameter<double>("alpha3", 0.05);
    alpha4_ = declare_parameter<double>("alpha4", 0.01);
    sigma_hit_ = declare_parameter<double>("sigma_hit", 0.15);
    max_dist_ = declare_parameter<double>("max_dist", 1.0);
    max_beams_ = declare_parameter<int>("max_beams", 120);
    use_kld_ = declare_parameter<bool>("use_kld", true);

    set_initial_pose_ = declare_parameter<bool>("set_initial_pose", true);
    init_x_ = declare_parameter<double>("initial_x", 0.0);
    init_y_ = declare_parameter<double>("initial_y", 0.0);
    init_yaw_ = declare_parameter<double>("initial_yaw", 0.0);
    const double init_sigma_xy = declare_parameter<double>("initial_sigma_xy", 0.3);
    const double init_sigma_yaw = declare_parameter<double>("initial_sigma_yaw", 0.2);

    pf_ = std::make_unique<ParticleFilter>(max_particles, 20241108);
    if (use_kld_) {
      pf_->setKldParams(kld_err, kld_z, min_particles, 0.10, 0.10);
    }
    pf_->setRecoveryParams(alpha_slow_, alpha_fast_);

    // ★ recovery（随机注入）默认【关闭】
    //   它是给"绑架机器人"场景用的。正常定位时开着它，一旦机器人走进
    //   地图没覆盖的区域（似然整体下滑），就会被误判成被绑架 →
    //   整张图撒随机粒子 → 位姿乱跳。
    //   要用的时候：-p enable_recovery:=true
    const bool enable_recovery = declare_parameter<bool>("enable_recovery", false);

    // 随机撒粒子：在地图范围内均匀（只有开了 recovery 才注册）
    if (enable_recovery) {
    pf_->setUniformPoseFn([this]() {
        Pose2D p;
        if (!has_map_) {return p;}
        std::uniform_real_distribution<double> ux(map_min_x_, map_max_x_);
        std::uniform_real_distribution<double> uy(map_min_y_, map_max_y_);
        std::uniform_real_distribution<double> ua(-M_PI, M_PI);
        p.x = ux(rng_); p.y = uy(rng_); p.yaw = ua(rng_);
        return p;
    });
    }

    pf_->initialize(init_x_, init_y_, init_yaw_, init_sigma_xy, init_sigma_yaw);

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(1).transient_local().reliable(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr m) {onMap(m);});

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr m) {onScan(m);});

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr m) {onOdom(m);});

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", 10);

    // 订阅 /initialpose：RViz 的 "2D Pose Estimate" 按钮发到这个话题
    //   （也可以命令行 ros2 topic pub --once /initialpose ...）
    //   收到后在【运行时】重新撒粒子，不用重启节点。
    initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", rclcpp::QoS(1),
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m) {onInitialPose(m);});

    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    RCLCPP_INFO(get_logger(),
      "amcl_mini 启动：%s + %s -> /amcl_pose，粒子数 %d~%d",
      scan_topic_.c_str(), map_topic_.c_str(), min_particles, max_particles);
  }

private:
  void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    const int w = static_cast<int>(msg->info.width);
    const int h = static_cast<int>(msg->info.height);
    const double res = msg->info.resolution;
    const double ox = msg->info.origin.position.x;
    const double oy = msg->info.origin.position.y;

    OccupancyGrid g(res, ox, oy, w, h);
    for (int gy = 0; gy < h; ++gy) {
      for (int gx = 0; gx < w; ++gx) {
        g.setOccupancyValue(gx, gy, msg->data[static_cast<size_t>(gy) * w + gx]);
      }
    }
    pf_->setMap(g);
    map_min_x_ = ox;
    map_min_y_ = oy;
    map_max_x_ = ox + w * res;
    map_max_y_ = oy + h * res;
    has_map_ = true;

    RCLCPP_INFO(get_logger(), "收到地图：%dx%d @ %.3f m，范围 x[%.2f, %.2f] y[%.2f, %.2f]",
      w, h, res, map_min_x_, map_max_x_, map_min_y_, map_max_y_);
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const Pose2D now_pose = poseFromMsg(*msg);
    if (!has_odom_) {
      last_odom_ = now_pose;
      has_odom_ = true;
      return;
    }
    pending_dx_ += now_pose.x - last_odom_.x;
    pending_dy_ += now_pose.y - last_odom_.y;
    pending_dyaw_ = wrapToPi(pending_dyaw_ + (now_pose.yaw - last_odom_.yaw));
    last_odom_ = now_pose;
  }

  static double yawFromQuat(const geometry_msgs::msg::Quaternion & q)
  {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  /// 收到新的初始位姿：重新初始化粒子滤波器
  void onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    const auto & p = msg->pose.pose.position;
    const double yaw = yawFromQuat(msg->pose.pose.orientation);

    // 协方差 → sigma。RViz 拖的方向越长，协方差越大，粒子撒得越开。
    // 开根号是因为协方差是方差（sigma^2）。下限 1e-4 防止 0 导致粒子全重合。
    const double sigma_xy = std::sqrt(std::max(msg->pose.covariance[0], 1e-4));
    const double sigma_yaw = std::sqrt(std::max(msg->pose.covariance[35], 1e-4));

    pf_->initialize(p.x, p.y, yaw, sigma_xy, sigma_yaw);

    // ★ 清掉还没消费的 odom 增量，否则重定位后第一帧会把旧位移加回去
    pending_dx_ = pending_dy_ = pending_dyaw_ = 0.0;

    RCLCPP_INFO(get_logger(),
      "收到新的初始位姿 (%.2f, %.2f, %.3f)，sigma=(%.2f, %.2f)，粒子已重新撒开",
      p.x, p.y, yaw, sigma_xy, sigma_yaw);
  }

  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (!has_map_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "还没收到 %s，跳过", map_topic_.c_str());
      return;
    }

    // 1. 用 odom 增量预测
    //    ★ 关键：ParticleFilter::predict() 期望的是【车体系】增量，
    //      而 odom 给的是【odom 系】增量。必须用上一刻的 odom 朝向把它转过来，
    //      否则一旦转向，整段位移会被多旋转一个 yaw。
    //      真实 AMCL 里这一步叫 delta_bearing = atan2(dy,dx) - old_odom_yaw
    if (pending_dx_ != 0.0 || pending_dy_ != 0.0 || pending_dyaw_ != 0.0) {
      const double c = std::cos(last_odom_.yaw);
      const double s = std::sin(last_odom_.yaw);
      const double body_dx =  c * pending_dx_ + s * pending_dy_;
      const double body_dy = -s * pending_dx_ + c * pending_dy_;

      pf_->predict(body_dx, body_dy, pending_dyaw_, alpha1_, alpha2_, alpha3_, alpha4_);
      pending_dx_ = pending_dy_ = pending_dyaw_ = 0.0;
    }

    // 2. 抽稀激光
    std::vector<float> ranges;
    const size_t n = msg->ranges.size();
    const int step = std::max(1, static_cast<int>(n) / std::max(1, max_beams_));
    for (size_t i = 0; i < n; i += static_cast<size_t>(step)) {
      ranges.push_back(msg->ranges[i]);
    }
    // 抽稀后角度间隔变了，要乘回去
    const double inc = msg->angle_increment * step;

    // 3. 加权 + 重采样
    pf_->updateWeights(ranges, msg->angle_min, inc, msg->range_max, sigma_hit_, max_dist_);
    pf_->resample();

    // 4. 发布
    publish(msg->header.stamp);
  }

  void publish(const rclcpp::Time & stamp)
  {
    const Pose2D est = pf_->estimatedPose();

    geometry_msgs::msg::PoseWithCovarianceStamped out;
    out.header.stamp = stamp;
    out.header.frame_id = map_frame_;
    out.pose.pose.position.x = est.x;
    out.pose.pose.position.y = est.y;
    out.pose.pose.orientation = quatFromYaw(est.yaw);
    
    // ★ 用粒子云的真实方差，而不是写死的值。
    //   两级 EKF 靠这个决定"该多信任 AMCL" —— 粒子散开时 R 大，
    //   EKF 就会更多依赖里程计预测；粒子收敛时 R 小，更信观测。
    // 下限 1e-4：粒子全部重合（方差 0）时如果原样发出去，
    //   EKF 会认为"这次观测绝对准确"，恰恰是最危险的情况。
    double var_x = 0.0, var_y = 0.0, var_yaw = 0.0;
    pf_->poseCovariance(var_x, var_y, var_yaw);
    out.pose.covariance[0]  = std::max(var_x, 1e-4);
    out.pose.covariance[7]  = std::max(var_y, 1e-4);
    out.pose.covariance[35] = std::max(var_yaw, 1e-4);

    pose_pub_->publish(out);

    if (tf_broadcaster_ && has_odom_) {
      // map->odom = (map->base) * inv(odom->base)
      const double c = std::cos(est.yaw), s = std::sin(est.yaw);
      const double ix = -std::cos(last_odom_.yaw) * last_odom_.x -
        std::sin(last_odom_.yaw) * last_odom_.y;
      const double iy = std::sin(last_odom_.yaw) * last_odom_.x -
        std::cos(last_odom_.yaw) * last_odom_.y;

      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp;
      tf.header.frame_id = map_frame_;
      tf.child_frame_id = odom_frame_;
      tf.transform.translation.x = est.x + c * ix - s * iy;
      tf.transform.translation.y = est.y + s * ix + c * iy;
      tf.transform.rotation = quatFromYaw(wrapToPi(est.yaw - last_odom_.yaw));
      tf_broadcaster_->sendTransform(tf);
    }
  }

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

  std::string scan_topic_, map_topic_, odom_topic_;
  std::string map_frame_, odom_frame_, base_frame_;
  bool publish_tf_{true};
  double alpha_slow_{0.001}, alpha_fast_{0.1};
  double alpha1_{0.05}, alpha2_{0.01}, alpha3_{0.05}, alpha4_{0.01};
  double sigma_hit_{0.15}, max_dist_{1.0};
  int max_beams_{120};
  bool use_kld_{true};
  bool set_initial_pose_{true};
  double init_x_{0.0}, init_y_{0.0}, init_yaw_{0.0};

  std::unique_ptr<ParticleFilter> pf_;
  std::mt19937 rng_;

  bool has_map_{false}, has_odom_{false};
  double map_min_x_{0}, map_max_x_{0}, map_min_y_{0}, map_max_y_{0};
  Pose2D last_odom_;
  double pending_dx_{0}, pending_dy_{0}, pending_dyaw_{0};

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace mini_slam

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mini_slam::AmclMiniNode>());
  rclcpp::shutdown();
  return 0;
}
