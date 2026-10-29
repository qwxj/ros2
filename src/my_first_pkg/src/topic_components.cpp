
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

rclcpp::NodeOptions enable_intra_process(
    const rclcpp::NodeOptions & options)
{
    auto new_options = options;
    new_options.use_intra_process_comms(true);
    return new_options;
}

class DemoPublisher : public rclcpp::Node
{
public:
    explicit DemoPublisher(const rclcpp::NodeOptions & options)
    : Node("demo_publisher", enable_intra_process(options)),
      count_(0)
    {
        publisher_ =
            this->create_publisher<std_msgs::msg::String>(
                "/component_topic", 10);

        timer_ = this->create_wall_timer(
            1s, [this]() {
                // std_msgs::msg::String msg;
                // msg.data = "组件消息 #" + std::to_string(count_++);

                // publisher_->publish(msg);

                auto msg = std::make_unique<std_msgs::msg::String>();

                msg->data = "组件消息 #" + std::to_string(count_++);

                RCLCPP_INFO(
                    this->get_logger(),
                    "发布：%s", msg->data.c_str());

                publisher_->publish(std::move(msg));
            });
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int count_;
};

class DemoSubscriber : public rclcpp::Node
{
public:
    explicit DemoSubscriber(const rclcpp::NodeOptions & options)
    : Node("demo_subscriber", enable_intra_process(options))
    {
        subscription_ =
            this->create_subscription<std_msgs::msg::String>(
                "/component_topic", 10,
                [this](std_msgs::msg::String::UniquePtr msg) {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "收到：%s", msg->data.c_str());
                });
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(DemoPublisher)
RCLCPP_COMPONENTS_REGISTER_NODE(DemoSubscriber)
