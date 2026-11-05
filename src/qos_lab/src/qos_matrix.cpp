// L1 实验一：QoS 兼容性矩阵 + depth 丢包 + deadline 违约。
//
// 一个进程里同时起 pub 和 sub，两端 QoS 各自来自参数，所以：
//   - 改 pub_/sub_ 的 reliability、durability 就能遍历兼容性矩阵
//   - matched pubs/subs 是否变成 1，直接告诉你"连没连上"
//   - sub_busy_ms 让订阅端变慢，配合 depth=1 压出 lost
//   - 设 sub_deadline_ms 后，gap 超过它就计一次违约
//
// 注意：为了让"慢订阅者"不拖住发布定时器，这里用了 MultiThreadedExecutor +
// 两个回调组。这是 L2 的内容，先看个眼熟。
#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <memory>
#include <string>
#include <thread>

#include "qos_lab/qos_utils.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"

using namespace std::chrono_literals;

class QosMatrix : public rclcpp::Node
{
public:
  QosMatrix()
  : rclcpp::Node("qos_matrix")
  {
    const std::string topic = declare_parameter<std::string>("topic", "qos_matrix_topic");
    const double rate_hz = declare_parameter<double>("rate_hz", 10.0);
    sub_busy_ms_ = declare_parameter<double>("sub_busy_ms", 0.0);

    rclcpp::QoS pub_qos = qos_lab::declare_qos(*this, "pub_");
    rclcpp::QoS sub_qos = qos_lab::declare_qos(*this, "sub_");

    pub_deadline_ms_ = declare_parameter<double>("pub_deadline_ms", 0.0);
    sub_deadline_ms_ = declare_parameter<double>("sub_deadline_ms", 0.0);

    if (pub_deadline_ms_ > 0.0) {
      pub_qos.deadline(rclcpp::Duration::from_seconds(pub_deadline_ms_ / 1000.0));
    }
    if (sub_deadline_ms_ > 0.0) {
      sub_qos.deadline(rclcpp::Duration::from_seconds(sub_deadline_ms_ / 1000.0));
    }

    RCLCPP_INFO(get_logger(), "topic=%s  rate=%.1f Hz  sub_busy=%.1f ms",
      topic.c_str(), rate_hz, sub_busy_ms_);
    RCLCPP_INFO(get_logger(), "  pub: %s", qos_lab::describe(pub_qos).c_str());
    RCLCPP_INFO(get_logger(), "  sub: %s", qos_lab::describe(sub_qos).c_str());

    timer_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    pub_ = create_publisher<std_msgs::msg::UInt64>(topic, pub_qos);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = sub_group_;
    sub_ = create_subscription<std_msgs::msg::UInt64>(
      topic, sub_qos,
      [this](std_msgs::msg::UInt64::SharedPtr msg) {on_msg(*msg);},
      sub_options);

    const auto period = std::chrono::milliseconds(
      std::max(1, static_cast<int>(1000.0 / rate_hz)));
    publish_timer_ = create_wall_timer(
      period, [this]() {
        std_msgs::msg::UInt64 msg;
        msg.data = seq_++;
        pub_->publish(msg);
      }, timer_group_);

    stats_timer_ = create_wall_timer(1s, [this]() {print_stats();}, timer_group_);
  }

private:
  void on_msg(const std_msgs::msg::UInt64 & msg)
  {
    if (sub_busy_ms_ > 0.0) {
      std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(sub_busy_ms_));
    }

    const rclcpp::Time stamp = now();
    if (have_last_) {
      const double gap_ms = (stamp - last_time_).seconds() * 1000.0;
      max_gap_ms_ = std::max(max_gap_ms_, gap_ms);
      if (sub_deadline_ms_ > 0.0 && gap_ms > sub_deadline_ms_) {
        ++deadline_violations_;
      }
    }
    last_time_ = stamp;
    have_last_ = true;

    if (received_ == 0) {
      expected_ = msg.data + 1;
    } else if (msg.data > expected_) {
      lost_ += msg.data - expected_;
      expected_ = msg.data + 1;
    } else if (msg.data == expected_) {
      expected_ = msg.data + 1;
    }
    ++received_;
  }

  void print_stats()
  {
    RCLCPP_INFO(
      get_logger(),
      "[stats] matched pubs=%zu subs=%zu | sent=%" PRIu64 " recv=%" PRIu64
      " lost=%" PRIu64 " | max_gap=%.1f ms deadline_viol=%" PRIu64,
      sub_->get_publisher_count(), pub_->get_subscription_count(),
      seq_, received_, lost_, max_gap_ms_, deadline_violations_);
  }

  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  rclcpp::CallbackGroup::SharedPtr timer_group_;
  rclcpp::CallbackGroup::SharedPtr sub_group_;

  double pub_deadline_ms_{0.0};
  double sub_deadline_ms_{0.0};
  double sub_busy_ms_{0.0};
  uint64_t seq_{0};
  uint64_t received_{0};
  uint64_t lost_{0};
  uint64_t expected_{0};
  uint64_t deadline_violations_{0};
  double max_gap_ms_{0.0};
  bool have_last_{false};
  rclcpp::Time last_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<QosMatrix>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
