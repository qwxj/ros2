
#include <rclcpp/rclcpp.hpp>
#include "my_robot_interfaces/msg/big_data.hpp"

#include <chrono>
#include <cstdint>
#include <deque>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>
#include <algorithm>
#include <thread>

using BigData = my_robot_interfaces::msg::BigData;
using Clock = std::chrono::steady_clock;

// ============================================================
// 1. Message Pool：消息对象池
// ============================================================

class MessagePool
{
public:
    MessagePool(size_t pool_size, size_t payload_size)
        : pool_size_(pool_size),
          payload_size_(payload_size)
    {
        // 程序启动时，提前创建所有消息对象
        for (size_t i = 0; i < pool_size_; ++i) {
            auto msg = std::make_unique<BigData>();

            msg->payload.resize(payload_size_);

            free_queue_.push_back(std::move(msg));
        }

        RCLCPP_INFO(
            rclcpp::get_logger("message_pool"),
            "Pool initialized: objects=%zu, payload=%.2f MB, total=%.2f MB",
            pool_size_,
            payload_size_ / 1024.0 / 1024.0,
            pool_size_ * payload_size_ / 1024.0 / 1024.0);
    }

    // 从对象池借出一个消息对象
    BigData::UniquePtr acquire()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (free_queue_.empty()) {
            acquire_failed_++;
            return nullptr;
        }

        auto msg = std::move(free_queue_.front());
        free_queue_.pop_front();

        acquired_count_++;

        return msg;
    }

    // 将消息对象归还对象池
    void release(BigData::UniquePtr msg)
    {
        if (!msg) {
            return;
        }

        // 清理本次消息的逻辑状态，但不释放 payload 容量
        msg->seq = 0;
        msg->send_time_ns = 0;

        std::lock_guard<std::mutex> lock(mutex_);

        free_queue_.push_back(std::move(msg));
        released_count_++;
    }

    size_t availableCount() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return free_queue_.size();
    }

    uint64_t acquireFailedCount() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return acquire_failed_;
    }

    uint64_t acquiredCount() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return acquired_count_;
    }

    uint64_t releasedCount() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return released_count_;
    }

private:
    size_t pool_size_;
    size_t payload_size_;

    mutable std::mutex mutex_;

    std::deque<BigData::UniquePtr> free_queue_;

    uint64_t acquired_count_{0};
    uint64_t released_count_{0};
    uint64_t acquire_failed_{0};
};

// ============================================================
// 2. Publisher
// ============================================================

class BigDataPublisher : public rclcpp::Node
{
public:
    BigDataPublisher(
        std::shared_ptr<MessagePool> pool,
        double frequency)
        : Node(
            "bigdata_pool_publisher",
            rclcpp::NodeOptions()
                .use_intra_process_comms(true)),
          pool_(std::move(pool))
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(5));
        qos.best_effort();

        publisher_ = create_publisher<BigData>(
            "/bigdata_pool_topic", qos);

        auto period =
            std::chrono::duration<double>(1.0 / frequency);

        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                period),
            std::bind(
                &BigDataPublisher::publishData,
                this));

        RCLCPP_INFO(
            get_logger(),
            "Publisher started: frequency=%.1f Hz",
            frequency);
    }

private:
    static uint64_t nowNs()
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
            Clock::now().time_since_epoch()).count();
    }

    void publishData()
    {
        // 1. 从对象池取出消息对象
        auto msg = pool_->acquire();

        if (!msg) {
            pool_exhausted_count_++;

            if (pool_exhausted_count_ % 10 == 1) {
                RCLCPP_WARN(
                    get_logger(),
                    "Message pool exhausted, skipped=%llu",
                    static_cast<unsigned long long>(
                        pool_exhausted_count_));
            }

            return;
        }

        // 2. 填充消息
        msg->seq = seq_++;
        msg->send_time_ns = nowNs();

        // payload 已经在对象池初始化时分配
        // 这里不再 resize，也不重新申请 1MB 内存
        if (!msg->payload.empty()) {
            msg->payload[0] =
                static_cast<uint8_t>(msg->seq % 256);
        }

        const uint64_t current_seq = msg->seq;

        // 3. 转移所有权并发布
        publisher_->publish(std::move(msg));

        published_count_++;

        if (current_seq % 50 == 0) {
            RCLCPP_INFO(
                get_logger(),
                "Published seq=%llu, pool_available=%zu",
                static_cast<unsigned long long>(current_seq),
                pool_->availableCount());
        }
    }

    std::shared_ptr<MessagePool> pool_;

    rclcpp::Publisher<BigData>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    uint64_t seq_{0};
    uint64_t published_count_{0};
    uint64_t pool_exhausted_count_{0};
};

// ============================================================
// 3. Subscriber
// ============================================================

class BigDataSubscriber : public rclcpp::Node
{
public:
    explicit BigDataSubscriber(
        std::shared_ptr<MessagePool> pool)
        : Node(
            "bigdata_pool_subscriber",
            rclcpp::NodeOptions()
                .use_intra_process_comms(true)),
          pool_(std::move(pool))
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(5));
        qos.best_effort();

        subscription_ = create_subscription<BigData>(
            "/bigdata_pool_topic",
            qos,
            std::bind(
                &BigDataSubscriber::messageCallback,
                this,
                std::placeholders::_1));

        stats_timer_ = create_wall_timer(
            std::chrono::seconds(5),
            std::bind(
                &BigDataSubscriber::printStats,
                this));

        last_stats_time_ = Clock::now();

        RCLCPP_INFO(
            get_logger(),
            "Subscriber started");
    }

private:
    void messageCallback(BigData::UniquePtr msg)
    {
        if (!msg) {
            return;
        }

        std::this_thread::sleep_for(
        std::chrono::milliseconds(300));

        const uint64_t receive_time =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                Clock::now().time_since_epoch()).count();

        const double latency_ms =
            (receive_time - msg->send_time_ns) / 1e6;

        latency_sum_ms_ += latency_ms;

        if (latency_ms > max_latency_ms_) {
            max_latency_ms_ = latency_ms;
        }

        latencies_ms_.push_back(latency_ms);

        received_count_++;

        if (has_previous_seq_ &&
            msg->seq > previous_seq_ + 1) {
            missing_count_ +=
                msg->seq - previous_seq_ - 1;
        }

        previous_seq_ = msg->seq;
        has_previous_seq_ = true;

        // 关键：处理完成后，将消息对象归还对象池
        pool_->release(std::move(msg));
    }

    void printStats()
    {
        const auto now = Clock::now();

        const double elapsed =
            std::chrono::duration<double>(
                now - last_stats_time_).count();

        const double rate =
            elapsed > 0.0
                ? received_count_ / elapsed
                : 0.0;

        double avg_latency = 0.0;
        double p95_latency = 0.0;

        if (!latencies_ms_.empty()) {
            avg_latency =
                latency_sum_ms_ / latencies_ms_.size();

            auto sorted = latencies_ms_;

            std::sort(sorted.begin(), sorted.end());

            const size_t index =
                static_cast<size_t>(
                    0.95 * (sorted.size() - 1));

            p95_latency = sorted[index];
        }

        RCLCPP_INFO(
            get_logger(),
            "\n========== Pool Benchmark ==========\n"
            "received       : %zu\n"
            "receive rate   : %.2f Hz\n"
            "latency avg    : %.3f ms\n"
            "latency p95    : %.3f ms\n"
            "latency max    : %.3f ms\n"
            "missing seq    : %llu\n"
            "pool available : %zu\n"
            "pool acquired  : %llu\n"
            "pool released  : %llu\n"
            "pool failed    : %llu\n"
            "====================================",
            received_count_,
            rate,
            avg_latency,
            p95_latency,
            max_latency_ms_,
            static_cast<unsigned long long>(missing_count_),
            pool_->availableCount(),
            static_cast<unsigned long long>(pool_->acquiredCount()),
            static_cast<unsigned long long>(pool_->releasedCount()),
            static_cast<unsigned long long>(pool_->acquireFailedCount()));

        received_count_ = 0;
        missing_count_ = 0;
        latency_sum_ms_ = 0.0;
        max_latency_ms_ = 0.0;
        latencies_ms_.clear();

        last_stats_time_ = now;
    }

    std::shared_ptr<MessagePool> pool_;

    rclcpp::Subscription<BigData>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr stats_timer_;

    size_t received_count_{0};
    uint64_t missing_count_{0};

    uint64_t previous_seq_{0};
    bool has_previous_seq_{false};

    double latency_sum_ms_{0.0};
    double max_latency_ms_{0.0};

    std::vector<double> latencies_ms_;

    Clock::time_point last_stats_time_;
};

// ============================================================
// 4. Main
// ============================================================

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // 参数：
    // ros2 run my_first_pkg bigdata_pool_benchmark [size_MB] [frequency] [pool_size]

    const size_t size_mb =
        argc >= 2 ? std::stoul(argv[1]) : 1;

    const double frequency =
        argc >= 3 ? std::stod(argv[2]) : 10.0;

    const size_t pool_size =
        argc >= 4 ? std::stoul(argv[3]) : 5;

    if (size_mb == 0 || frequency <= 0.0 || pool_size == 0) {
        std::cerr << "Invalid arguments" << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    const size_t payload_size =
        size_mb * 1024ULL * 1024ULL;

    // 创建共享对象池
    auto pool = std::make_shared<MessagePool>(
        pool_size,
        payload_size);

    // 节点在 main 作用域内持有，保证生命周期
    auto publisher =
        std::make_shared<BigDataPublisher>(
            pool,
            frequency);

    auto subscriber =
        std::make_shared<BigDataSubscriber>(pool);

    rclcpp::executors::SingleThreadedExecutor executor;

    executor.add_node(publisher);
    executor.add_node(subscriber);

    RCLCPP_INFO(
        rclcpp::get_logger("main"),
        "Pool benchmark started: size=%zu MB, frequency=%.1f Hz, pool=%zu",
        size_mb,
        frequency,
        pool_size);

    executor.spin();

    executor.remove_node(publisher);
    executor.remove_node(subscriber);

    rclcpp::shutdown();

    return 0;
}
