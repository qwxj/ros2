
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;

class QosPublisher : public rclcpp::Node
{
public:
    QosPublisher()
        : Node("qos_publisher"), count_(0)
    {
        // 设置历史记录：最多保留最近 10 条消息
        // 设置可靠性：Best Effort
        auto qos = rclcpp::QoS(
            rclcpp::KeepLast(10)).reliable();

        publisher_ =
            this->create_publisher<std_msgs::msg::String>(
                "qos_topic", qos);

        timer_ = this->create_wall_timer(
            1s,
            [this]()
            {
                std_msgs::msg::String msg;
                msg.data = "QoS test: " +
                           std::to_string(count_++);

                publisher_->publish(msg);

                RCLCPP_INFO(
                    this->get_logger(),
                    "Published: '%s'",
                    msg.data.c_str());
            });
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<QosPublisher>());

    rclcpp::shutdown();
    return 0;
}

