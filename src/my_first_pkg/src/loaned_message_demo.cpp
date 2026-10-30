
#include <rclcpp/rclcpp.hpp>
#include "my_robot_interfaces/msg/loaned_data.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <utility>

using LoanedData = my_robot_interfaces::msg::LoanedData;
using namespace std::chrono_literals;

// ============================================================
// Publisher
// ============================================================
class LoanedPublisher : public rclcpp::Node
{
public:
    LoanedPublisher()
        : Node("loaned_publisher")
    {
        publisher_ = create_publisher<LoanedData>(
            "/loaned_topic",
            rclcpp::QoS(10)
        );

        // 检查底层 RMW 是否支持真正的消息借用
        can_loan_ = publisher_->can_loan_messages();

        RCLCPP_INFO(
            get_logger(),
            "Can loan messages: %s",
            can_loan_ ? "YES" : "NO"
        );

        timer_ = create_wall_timer(
            100ms,
            std::bind(&LoanedPublisher::publishData, this)
        );

        RCLCPP_INFO(get_logger(), "Loaned Publisher started");
    }

private:
    void publishData()
    {
        // 向 Publisher 申请一个 Loaned Message
        auto loaned_msg = publisher_->borrow_loaned_message();

        // 获取消息对象并填写数据
        auto & msg = loaned_msg.get();

        msg.seq = seq_++;
        msg.send_time_ns = getTimeNs();
        msg.value = static_cast<uint8_t>(msg.seq % 256);

        RCLCPP_INFO(
            get_logger(),
            "Publishing seq=%lu, value=%u",
            static_cast<unsigned long>(msg.seq),
            static_cast<unsigned int>(msg.value)
        );

        // 发布并将借用资源的所有权交还给发布机制
        publisher_->publish(std::move(loaned_msg));
    }

    static uint64_t getTimeNs()
    {
        return static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()
            ).count()
        );
    }

private:
    rclcpp::Publisher<LoanedData>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    uint64_t seq_{0};
    bool can_loan_{false};
};

// ============================================================
// Subscriber
// ============================================================
class LoanedSubscriber : public rclcpp::Node
{
public:
    LoanedSubscriber()
        : Node("loaned_subscriber")
    {
        subscription_ = create_subscription<LoanedData>(
            "/loaned_topic",
            rclcpp::QoS(10),
            std::bind(
                &LoanedSubscriber::messageCallback,
                this,
                std::placeholders::_1
            )
        );

        RCLCPP_INFO(get_logger(), "Loaned Subscriber started");
    }

private:
    void messageCallback(const LoanedData::SharedPtr msg)
    {
        RCLCPP_INFO(
            get_logger(),
            "Received seq=%lu, value=%u",
            static_cast<unsigned long>(msg->seq),
            static_cast<unsigned int>(msg->value)
        );
    }

private:
    rclcpp::Subscription<LoanedData>::SharedPtr subscription_;
};

// ============================================================
// main
// ============================================================
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto publisher = std::make_shared<LoanedPublisher>();
    auto subscriber = std::make_shared<LoanedSubscriber>();

    rclcpp::executors::SingleThreadedExecutor executor;

    executor.add_node(publisher);
    executor.add_node(subscriber);

    executor.spin();

    rclcpp::shutdown();

    return 0;
}
