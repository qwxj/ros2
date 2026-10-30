#include <rclcpp/rclcpp.hpp>
#include "my_robot_interfaces/msg/loaned_data.hpp"

#include <chrono>
#include <algorithm>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <numeric>
#include <functional>

using namespace std::chrono_literals;
using LoanedData = my_robot_interfaces::msg::LoanedData;

static uint64_t nowNs()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()
    ).count();
}

// 发布端
class ComparePublisher : public rclcpp::Node
{
public:
    ComparePublisher(const std::string &mode, double frequency)
        : Node("compare_publisher"),
          mode_(mode),
          seq_(0)
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                       .best_effort();

        publisher_ = create_publisher<LoanedData>(
            "/loaned_topic", qos);

        RCLCPP_INFO(
            get_logger(),
            "Mode: %s, frequency: %.1f Hz",
            mode_.c_str(), frequency);

        RCLCPP_INFO(
            get_logger(),
            "Can loan messages: %s",
            publisher_->can_loan_messages() ? "YES" : "NO");

        auto period = std::chrono::duration<double>(
            1.0 / frequency);

        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&ComparePublisher::publishData, this));
    }

private:
    void publishData()
    {
        const uint64_t timestamp = nowNs();

        if (mode_ == "loaned")
        {
            auto loaned_msg = publisher_->borrow_loaned_message();

            auto &msg = loaned_msg.get();
            msg.seq = seq_;
            msg.send_time_ns = timestamp;
            msg.value = static_cast<uint8_t>(seq_ % 256);

            publisher_->publish(std::move(loaned_msg));
        }
        else
        {
            LoanedData msg;
            msg.seq = seq_;
            msg.send_time_ns = timestamp;
            msg.value = static_cast<uint8_t>(seq_ % 256);

            publisher_->publish(msg);
        }

        if (seq_ % 1000 == 0)
        {
            RCLCPP_INFO(
                get_logger(),
                "Published seq=%lu",
                static_cast<unsigned long>(seq_));
        }

        ++seq_;
    }

    std::string mode_;
    uint64_t seq_;

    rclcpp::Publisher<LoanedData>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// 订阅端
class CompareSubscriber : public rclcpp::Node
{
public:
    CompareSubscriber()
        : Node("compare_subscriber"),
          received_(0)
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                       .best_effort();

        subscription_ = create_subscription<LoanedData>(
            "/loaned_topic",
            qos,
            std::bind(
                &CompareSubscriber::receiveData,
                this,
                std::placeholders::_1));

        stats_timer_ = create_wall_timer(
            5s,
            std::bind(&CompareSubscriber::printStats, this));

        RCLCPP_INFO(get_logger(), "Subscriber started");
    }

private:
    void receiveData(const LoanedData::SharedPtr msg)
    {
        const uint64_t current = nowNs();

        const double latency_ms =
            (current - msg->send_time_ns) / 1e6;

        latencies_.push_back(latency_ms);
        ++received_;
    }

    void printStats()
    {
        if (latencies_.empty())
        {
            RCLCPP_WARN(get_logger(), "No messages received");
            return;
        }

        auto sorted = latencies_;
        std::sort(sorted.begin(), sorted.end());

        const double sum = std::accumulate(
            sorted.begin(), sorted.end(), 0.0);

        const double avg = sum / sorted.size();

        const size_t p95_index =
            static_cast<size_t>(0.95 * (sorted.size() - 1));

        RCLCPP_INFO(get_logger(), "========== Statistics ==========");
        RCLCPP_INFO(
            get_logger(), "Received: %lu",
            static_cast<unsigned long>(received_));
        RCLCPP_INFO(
            get_logger(), "Receive rate: %.2f Hz",
            received_ / 5.0);
        RCLCPP_INFO(
            get_logger(), "Latency avg: %.3f ms", avg);
        RCLCPP_INFO(
            get_logger(), "Latency p95: %.3f ms",
            sorted[p95_index]);
        RCLCPP_INFO(
            get_logger(), "Latency max: %.3f ms",
            sorted.back());

        latencies_.clear();
        received_ = 0;
    }

    uint64_t received_;
    std::vector<double> latencies_;

    rclcpp::Subscription<LoanedData>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::string mode = "normal";
    double frequency = 1000.0;

    if (argc >= 2)
        mode = argv[1];

    if (argc >= 3)
        frequency = std::stod(argv[2]);

    if (mode != "normal" && mode != "loaned")
    {
        std::cerr << "Mode must be normal or loaned\n";
        rclcpp::shutdown();
        return 1;
    }

    if (frequency <= 0.0)
    {
        std::cerr << "Frequency must be positive\n";
        rclcpp::shutdown();
        return 1;
    }

    auto pub = std::make_shared<ComparePublisher>(
        mode, frequency);

    auto sub = std::make_shared<CompareSubscriber>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(pub);
    executor.add_node(sub);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}