
#include <rclcpp/rclcpp.hpp>
#include "my_robot_interfaces/msg/big_loaned_data.hpp"

#include <chrono>
#include <algorithm>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

using namespace std::chrono_literals;

using BigLoanedData =
    my_robot_interfaces::msg::BigLoanedData;

// 获取单调时钟纳秒时间戳
static uint64_t nowNs()
{
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()
        ).count()
    );
}

// ============================================================
// Publisher
// ============================================================

class BigComparePublisher : public rclcpp::Node
{
public:
    BigComparePublisher(
        const std::string &mode,
        double frequency)
        : Node("big_compare_publisher"),
          mode_(mode),
          frequency_(frequency),
          seq_(0)
    {
        auto qos = rclcpp::QoS(
            rclcpp::KeepLast(10)).best_effort();

        publisher_ = create_publisher<BigLoanedData>(
            "/loaned_topic",
            qos
        );

        RCLCPP_INFO(
            get_logger(),
            "====================================");

        RCLCPP_INFO(
            get_logger(),
            "Mode: %s",
            mode_.c_str());

        RCLCPP_INFO(
            get_logger(),
            "Frequency: %.1f Hz",
            frequency_);

        RCLCPP_INFO(
            get_logger(),
            "Payload size: 1 MB");

        RCLCPP_INFO(
            get_logger(),
            "Can loan messages: %s",
            publisher_->can_loan_messages() ? "YES" : "NO");

        auto period = std::chrono::duration<double>(
            1.0 / frequency_);

        timer_ = create_wall_timer(
            std::chrono::duration_cast<
                std::chrono::nanoseconds>(period),
            std::bind(
                &BigComparePublisher::publishData,
                this)
        );

        RCLCPP_INFO(
            get_logger(),
            "Publisher started");
    }

private:
    void publishData()
    {
        const uint64_t timestamp = nowNs();

        if (mode_ == "loaned")
        {
            // 从中间件借用消息对象
            auto loaned_msg =
                publisher_->borrow_loaned_message();

            auto &msg = loaned_msg.get();

            msg.seq = seq_;
            msg.send_time_ns = timestamp;

            // 填充完整的 1 MB 数据
            std::fill(
                msg.payload.begin(),
                msg.payload.end(),
                static_cast<uint8_t>(seq_ % 256)
            );

            publisher_->publish(
                std::move(loaned_msg));
        }
        else
        {
            // 普通消息发布
            BigLoanedData msg;

            msg.seq = seq_;
            msg.send_time_ns = timestamp;

            // 填充完整的 1 MB 数据
            std::fill(
                msg.payload.begin(),
                msg.payload.end(),
                static_cast<uint8_t>(seq_ % 256)
            );

            publisher_->publish(msg);
        }

        if (seq_ % 100 == 0)
        {
            RCLCPP_INFO(
                get_logger(),
                "Published seq=%lu",
                static_cast<unsigned long>(seq_));
        }

        ++seq_;
    }

    std::string mode_;
    double frequency_;
    uint64_t seq_;

    rclcpp::Publisher<BigLoanedData>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// ============================================================
// Subscriber
// ============================================================

class BigCompareSubscriber : public rclcpp::Node
{
public:
    BigCompareSubscriber()
        : Node("big_compare_subscriber"),
          received_(0),
          total_received_(0),
          last_seq_(0),
          has_last_seq_(false),
          missing_seq_(0),
          last_stats_time_(nowNs())
    {
        auto qos = rclcpp::QoS(
            rclcpp::KeepLast(10)).best_effort();

        subscription_ =
            create_subscription<BigLoanedData>(
                "/loaned_topic",
                qos,
                std::bind(
                    &BigCompareSubscriber::receiveData,
                    this,
                    std::placeholders::_1)
            );

        stats_timer_ = create_wall_timer(
            5s,
            std::bind(
                &BigCompareSubscriber::printStats,
                this)
        );

        RCLCPP_INFO(
            get_logger(),
            "Subscriber started");
    }

private:
    void receiveData(
        const BigLoanedData::SharedPtr msg)
    {
        const uint64_t current = nowNs();

        const double latency_ms =
            static_cast<double>(
                current - msg->send_time_ns
            ) / 1e6;

        latencies_.push_back(latency_ms);

        ++received_;
        ++total_received_;

        // 统计观察到的序号间隙
        if (has_last_seq_)
        {
            if (msg->seq > last_seq_ + 1)
            {
                missing_seq_ +=
                    msg->seq - last_seq_ - 1;
            }
        }

        last_seq_ = msg->seq;
        has_last_seq_ = true;
    }

    void printStats()
    {
        if (latencies_.empty())
        {
            RCLCPP_WARN(
                get_logger(),
                "No messages received");

            return;
        }

        auto sorted = latencies_;

        std::sort(
            sorted.begin(),
            sorted.end());

        const double sum = std::accumulate(
            sorted.begin(),
            sorted.end(),
            0.0);

        const double avg =
            sum / static_cast<double>(sorted.size());

        const size_t p95_index =
            static_cast<size_t>(
                0.95 * (sorted.size() - 1));

        const double elapsed_seconds =
            static_cast<double>(
                nowNs() - last_stats_time_
            ) / 1e9;

        const double receive_rate =
            received_ / elapsed_seconds;

        RCLCPP_INFO(
            get_logger(),
            "========== Statistics ==========");

        RCLCPP_INFO(
            get_logger(),
            "Received: %lu",
            static_cast<unsigned long>(received_));

        RCLCPP_INFO(
            get_logger(),
            "Total received: %lu",
            static_cast<unsigned long>(total_received_));

        RCLCPP_INFO(
            get_logger(),
            "Receive rate: %.2f Hz",
            receive_rate);

        RCLCPP_INFO(
            get_logger(),
            "Latency avg: %.3f ms",
            avg);

        RCLCPP_INFO(
            get_logger(),
            "Latency p95: %.3f ms",
            sorted[p95_index]);

        RCLCPP_INFO(
            get_logger(),
            "Latency max: %.3f ms",
            sorted.back());

        RCLCPP_INFO(
            get_logger(),
            "Missing seq: %lu",
            static_cast<unsigned long>(missing_seq_));

        RCLCPP_INFO(
            get_logger(),
            "=================================");

        // 清理当前统计周期的数据
        latencies_.clear();
        received_ = 0;
        missing_seq_ = 0;
        last_stats_time_ = nowNs();
    }

    uint64_t received_;
    uint64_t total_received_;

    uint64_t last_seq_;
    bool has_last_seq_;

    uint64_t missing_seq_;

    uint64_t last_stats_time_;

    std::vector<double> latencies_;

    rclcpp::Subscription<BigLoanedData>::SharedPtr
        subscription_;

    rclcpp::TimerBase::SharedPtr stats_timer_;
};

// ============================================================
// Main
// ============================================================

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::string mode = "normal";
    double frequency = 100.0;

    if (argc >= 2)
    {
        mode = argv[1];
    }

    if (argc >= 3)
    {
        try
        {
            frequency = std::stod(argv[2]);
        }
        catch (...)
        {
            std::cerr << "Invalid frequency\n";
            rclcpp::shutdown();
            return 1;
        }
    }

    if (mode != "normal" && mode != "loaned")
    {
        std::cerr
            << "Usage: big_loaned_compare "
            << "[normal|loaned] [frequency]\n";

        rclcpp::shutdown();
        return 1;
    }

    if (frequency <= 0.0)
    {
        std::cerr << "Frequency must be positive\n";
        rclcpp::shutdown();
        return 1;
    }

    auto publisher =
        std::make_shared<BigComparePublisher>(
            mode,
            frequency);

    auto subscriber =
        std::make_shared<BigCompareSubscriber>();

    rclcpp::executors::SingleThreadedExecutor executor;

    executor.add_node(publisher);
    executor.add_node(subscriber);

    RCLCPP_INFO(
        rclcpp::get_logger("big_loaned_compare"),
        "Big Loaned Compare started");

    executor.spin();

    rclcpp::shutdown();

    return 0;
}
