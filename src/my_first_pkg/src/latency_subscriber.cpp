
#include <memory>
#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/latency_data.hpp"

using namespace std::chrono_literals;

class LatencySubscriber : public rclcpp::Node
{
public:
    LatencySubscriber()
        : Node("latency_subscriber")
    {
        // 创建 Subscriber
        subscription_ = this->create_subscription<
            my_robot_interfaces::msg::LatencyData>(
                "/latency_topic",
                1,
                std::bind(
                    &LatencySubscriber::callback,
                    this,
                    std::placeholders::_1));

        RCLCPP_INFO(
            this->get_logger(),
            "Latency Subscriber started");
    }

private:
    void callback(
        const my_robot_interfaces::msg::LatencyData::SharedPtr msg)
    {
        // 获取当前时间
        auto now = this->get_clock()->now();

        // 计算消息延迟
        double latency_ms =
            (now - rclcpp::Time(msg->stamp)).seconds() * 1000.0;

        RCLCPP_INFO(
            this->get_logger(),
            "Received seq=%lu, latency=%.3f ms, data=%s",
            msg->seq,
            latency_ms,
            msg->data.c_str());

        // 模拟耗时业务处理
        // 后续实验再打开
        rclcpp::sleep_for(300ms);
    }

    rclcpp::Subscription<
        my_robot_interfaces::msg::LatencyData>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<LatencySubscriber>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
