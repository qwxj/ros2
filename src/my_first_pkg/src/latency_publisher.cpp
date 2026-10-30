
#include <chrono>
#include <memory>
#include <functional>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/latency_data.hpp"

using namespace std::chrono_literals;

class LatencyPublisher : public rclcpp::Node
{
public:
    LatencyPublisher()
        : Node("latency_publisher"),
          seq_(0)
    {
        // 创建 Publisher
        publisher_ = this->create_publisher<
            my_robot_interfaces::msg::LatencyData>(
                "/latency_topic", 10);

        // 每隔 100ms 发布一次
        timer_ = this->create_wall_timer(
            100ms,
            std::bind(
                &LatencyPublisher::publish_data,
                this));

        RCLCPP_INFO(
            this->get_logger(),
            "Latency Publisher started");
    }

private:
    void publish_data()
    {
        auto msg =
            my_robot_interfaces::msg::LatencyData();

        // 消息序号
        msg.seq = seq_++;

        // 记录发布时间
        msg.stamp = this->get_clock()->now();

        // 测试数据
        msg.data = "ROS2 latency test";

        // 发布消息
        publisher_->publish(msg);

        RCLCPP_INFO(
            this->get_logger(),
            "Published seq=%lu",
            msg.seq);
    }

    rclcpp::Publisher<
        my_robot_interfaces::msg::LatencyData>::SharedPtr publisher_;

    rclcpp::TimerBase::SharedPtr timer_;

    uint64_t seq_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<LatencyPublisher>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
