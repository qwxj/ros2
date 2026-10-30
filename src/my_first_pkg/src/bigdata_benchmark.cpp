
#include <rclcpp/rclcpp.hpp>
#include "my_robot_interfaces/msg/big_data.hpp"

#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <numeric>
#include <algorithm>
#include <vector>
#include <string>
#include <functional>

using BigData = my_robot_interfaces::msg::BigData;
using Clock = std::chrono::steady_clock;

// 获取单调时钟纳秒时间戳
static uint64_t nowNs()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        Clock::now().time_since_epoch()).count();
}

// ============================================================
// Publisher
// ============================================================

class BigDataPublisher : public rclcpp::Node
{
public:
    BigDataPublisher(
        size_t payload_size,
        double frequency,
        bool intra_process)
        : Node(
            "bigdata_publisher",
            rclcpp::NodeOptions()
                .use_intra_process_comms(intra_process)),
          payload_size_(payload_size)
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(5));
        qos.best_effort();

        publisher_ = create_publisher<BigData>(
            "/bigdata_topic", qos);

        auto period =
            std::chrono::duration<double>(1.0 / frequency);

        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                period),
            [this]() {
                RCLCPP_INFO(
                    get_logger(),
                    "Publisher timer triggered");

                publishData();
            });

        RCLCPP_INFO(
            get_logger(),
            "Publisher started: payload=%.2f MB, frequency=%.1f Hz, intra=%s",
            payload_size_ / 1024.0 / 1024.0,
            frequency,
            intra_process ? "true" : "false");

        RCLCPP_INFO(
            get_logger(),
            "Publisher timer created");
    }

private:
    void publishData()
    {
        auto msg = std::make_unique<BigData>();

        msg->seq = seq_++;
        msg->send_time_ns = nowNs();
        msg->payload.resize(payload_size_);

        // 模拟图像、点云等大数据载荷
        if (!msg->payload.empty()) {
            msg->payload[0] =
                static_cast<uint8_t>(msg->seq % 256);
        }

        const uint64_t current_seq = msg->seq;

        publisher_->publish(std::move(msg));

        if (current_seq % 10 == 0) {
            RCLCPP_INFO(
                get_logger(),
                "Published seq=%llu",
                static_cast<unsigned long long>(current_seq));
        }
    }

    size_t payload_size_;
    uint64_t seq_{0};

    rclcpp::Publisher<BigData>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// ============================================================
// Subscriber
// ============================================================

class BigDataSubscriber : public rclcpp::Node
{
public:
    explicit BigDataSubscriber(bool intra_process)
        : Node(
            "bigdata_subscriber",
            rclcpp::NodeOptions()
                .use_intra_process_comms(intra_process)),
          intra_process_(intra_process)
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(5));
        qos.best_effort();

        if (intra_process_) {
            // Intra-process：UniquePtr 接收
            intra_sub_ = create_subscription<BigData>(
                "/bigdata_topic",
                qos,
                std::bind(
                    &BigDataSubscriber::intraCallback,
                    this,
                    std::placeholders::_1));
        } else {
            // DDS：SharedPtr 接收
            dds_sub_ = create_subscription<BigData>(
                "/bigdata_topic",
                qos,
                std::bind(
                    &BigDataSubscriber::ddsCallback,
                    this,
                    std::placeholders::_1));
        }

        stats_timer_ = create_wall_timer(
            std::chrono::seconds(5),
            std::bind(
                &BigDataSubscriber::printStats,
                this));

        last_stats_time_ = Clock::now();

        RCLCPP_INFO(
            get_logger(),
            "Subscriber started: intra=%s",
            intra_process_ ? "true" : "false");

        RCLCPP_INFO(
            get_logger(),
            "Subscriber statistics timer created");
    }

private:
    void intraCallback(BigData::UniquePtr msg)
    {
        processMessage(*msg);
    }

    void ddsCallback(const BigData::SharedPtr msg)
    {
        processMessage(*msg);
    }

    void processMessage(const BigData &msg)
    {
        const uint64_t receive_time = nowNs();

        const double latency_ms =
            static_cast<double>(
                receive_time - msg.send_time_ns) / 1e6;

        latencies_ms_.push_back(latency_ms);
        received_count_++;

        if (msg.seq % 10 == 0) {
            RCLCPP_INFO(
                get_logger(),
                "Received seq=%llu",
                static_cast<unsigned long long>(msg.seq));
        }

        if (has_previous_seq_ &&
            msg.seq > previous_seq_ + 1) {
            missing_count_ +=
                msg.seq - previous_seq_ - 1;
        }

        previous_seq_ = msg.seq;
        has_previous_seq_ = true;
    }

    void printStats()
    {
        const auto now = Clock::now();

        const double elapsed =
            std::chrono::duration<double>(
                now - last_stats_time_).count();

        const double rate =
            elapsed > 0.0
                ? static_cast<double>(received_count_) / elapsed
                : 0.0;

        if (latencies_ms_.empty()) {
            RCLCPP_WARN(
                get_logger(),
                "No messages received in %.2f seconds",
                elapsed);

            last_stats_time_ = now;
            return;
        }

        std::sort(
            latencies_ms_.begin(),
            latencies_ms_.end());

        const double sum =
            std::accumulate(
                latencies_ms_.begin(),
                latencies_ms_.end(),
                0.0);

        const double avg =
            sum / latencies_ms_.size();

        const size_t p95_index =
            static_cast<size_t>(
                0.95 * (latencies_ms_.size() - 1));

        const double p95 =
            latencies_ms_[p95_index];

        const double max =
            latencies_ms_.back();

        RCLCPP_INFO(
            get_logger(),
            "\n========== Benchmark Statistics ==========\n"
            "received       : %zu\n"
            "receive rate   : %.2f Hz\n"
            "latency avg    : %.3f ms\n"
            "latency p95    : %.3f ms\n"
            "latency max    : %.3f ms\n"
            "missing seq    : %llu\n"
            "==========================================",
            received_count_,
            rate,
            avg,
            p95,
            max,
            static_cast<unsigned long long>(missing_count_));

        received_count_ = 0;
        missing_count_ = 0;
        latencies_ms_.clear();
        last_stats_time_ = now;
    }

    bool intra_process_;

    rclcpp::Subscription<BigData>::SharedPtr intra_sub_;
    rclcpp::Subscription<BigData>::SharedPtr dds_sub_;

    rclcpp::TimerBase::SharedPtr stats_timer_;

    std::vector<double> latencies_ms_;

    size_t received_count_{0};
    uint64_t missing_count_{0};

    uint64_t previous_seq_{0};
    bool has_previous_seq_{false};

    Clock::time_point last_stats_time_;
};

// ============================================================
// Main
// ============================================================


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc < 2) {
        std::cout
            << "Usage:\n"
            << "  bigdata_benchmark intra [size_MB] [frequency]\n"
            << "  bigdata_benchmark dds [size_MB] [frequency]\n"
            << "  bigdata_benchmark ipc_pub [size_MB] [frequency]\n"
            << "  bigdata_benchmark ipc_sub\n";

        rclcpp::shutdown();
        return 0;
    }

    const std::string mode = argv[1];

    const size_t size_mb =
        argc >= 3 ? std::stoul(argv[2]) : 1;

    const double frequency =
        mode == "ipc_sub"
            ? 10.0
            : (argc >= 4 ? std::stod(argv[3]) : 10.0);

    if (mode != "ipc_sub" &&
        (frequency <= 0.0 || size_mb == 0)) {
        std::cerr << "Invalid size or frequency" << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    const size_t payload_size =
        size_mb * 1024ULL * 1024ULL;

    const bool intra_process =
        (mode == "intra");

    rclcpp::executors::SingleThreadedExecutor executor;

    // 关键修改：节点指针在 main() 作用域中声明
    // 保证它们在 executor.spin() 期间一直存活
    std::shared_ptr<BigDataPublisher> publisher;
    std::shared_ptr<BigDataSubscriber> subscriber;

    // Executor 心跳节点
    auto test_node =
        std::make_shared<rclcpp::Node>("executor_test");

    auto test_timer = test_node->create_wall_timer(
        std::chrono::seconds(1),
        []() {
            std::cout
                << "[EXECUTOR] Timer is running"
                << std::endl;
        });

    executor.add_node(test_node);

    if (mode == "intra" || mode == "dds") {

        publisher = std::make_shared<BigDataPublisher>(
            payload_size,
            frequency,
            intra_process);

        subscriber = std::make_shared<BigDataSubscriber>(
            intra_process);

        executor.add_node(publisher);
        executor.add_node(subscriber);
    }
    else if (mode == "ipc_pub") {

        publisher = std::make_shared<BigDataPublisher>(
            payload_size,
            frequency,
            false);

        executor.add_node(publisher);
    }
    else if (mode == "ipc_sub") {

        subscriber = std::make_shared<BigDataSubscriber>(
            false);

        executor.add_node(subscriber);
    }
    else {
        std::cerr
            << "Unknown mode: " << mode
            << std::endl;

        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(
        test_node->get_logger(),
        "Executor spinning...");

    executor.spin();

    // spin() 退出后再释放节点
    executor.remove_node(test_node);

    if (publisher) {
        executor.remove_node(publisher);
    }

    if (subscriber) {
        executor.remove_node(subscriber);
    }

    rclcpp::shutdown();

    return 0;
}
