// L1 实验二：lifespan + transient_local + 晚加入的订阅者。
//
// 发布端先跑 sub_delay_s 秒，然后才创建订阅：
//   - pub_lifespan_ms=0（无限）+ transient_local：订阅者一上来就收到囤积的 10 条历史
//   - pub_lifespan_ms=1000 + 3 s 后才订阅：历史全部过期，只能收到之后的实时消息
#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <memory>
#include <string>

#include "qos_lab/qos_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"

using namespace std::chrono_literals;

class LifespanLab : public rclcpp::Node
{
public:
  LifespanLab()
  : rclcpp::Node("lifespan_lab")
  {
    const std::string topic = declare_parameter<std::string>("topic", "lifespan_topic");
    const double rate_hz = declare_parameter<double>("rate_hz", 5.0);
    lifespan_ms_ = declare_parameter<double>("pub_lifespan_ms", 1000.0);
    const double sub_delay_s = declare_parameter<double>("sub_delay_s", 3.0);

    // 先声明默认值，再交给 declare_qos 读——顺便演示"参数已存在时不再重新声明"。
    declare_parameter<std::string>("pub_durability", "transient_local");
    declare_parameter<std::string>("sub_durability", "transient_local");
    declare_parameter<int>("pub_depth", 10);
    declare_parameter<int>("sub_depth", 10);

    rclcpp::QoS pub_qos = qos_lab::declare_qos(*this, "pub_");
    sub_qos_ = qos_lab::declare_qos(*this, "sub_");

    if (lifespan_ms_ > 0.0) {
      pub_qos.lifespan(rclcpp::Duration::from_seconds(lifespan_ms_ / 1000.0));
    }

    RCLCPP_INFO(get_logger(), "  pub: %s", qos_lab::describe(pub_qos).c_str());
    RCLCPP_INFO(get_logger(), "  sub: %s", qos_lab::describe(sub_qos_).c_str());
    RCLCPP_INFO(get_logger(), "订阅在 %.1f s 后才创建，先让发布端攒一点历史", sub_delay_s);

    pub_ = create_publisher<std_msgs::msg::UInt64>(topic, pub_qos);

    const auto period = std::chrono::milliseconds(
      std::max(1, static_cast<int>(1000.0 / rate_hz)));
    publish_timer_ = create_wall_timer(
      period, [this]() {
        std_msgs::msg::UInt64 msg;
        msg.data = seq_++;
        pub_->publish(msg);
      });

    t0_ = now();
    delay_timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(sub_delay_s * 1000.0)),
      [this, topic]() {
        delay_timer_->cancel();
        RCLCPP_INFO(
          get_logger(), "t=%.2f s 创建订阅，接下来若成批出现即为历史消息",
          seconds_since_start());
        sub_ = create_subscription<std_msgs::msg::UInt64>(
          topic, sub_qos_,
          [this](std_msgs::msg::UInt64::SharedPtr msg) {
            RCLCPP_INFO(
              get_logger(), "  recv seq=%" PRIu64 "  t=%.2f s",
              msg->data, seconds_since_start());
          });
      });
  }

private:
  double seconds_since_start()
  {
    return (now() - t0_).seconds();
  }

  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr delay_timer_;
  rclcpp::QoS sub_qos_{10};
  rclcpp::Time t0_;
  double lifespan_ms_{0.0};
  uint64_t seq_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LifespanLab>());
  rclcpp::shutdown();
  return 0;
}
