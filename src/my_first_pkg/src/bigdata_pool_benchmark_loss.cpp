
#include <rclcpp/rclcpp.hpp>
#include "my_robot_interfaces/msg/big_data.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <iomanip>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

using BigData = my_robot_interfaces::msg::BigData;
using namespace std::chrono_literals;

// 获取 steady_clock 纳秒时间戳
static uint64_t getTimeNs()
{
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()
        ).count()
    );
}

// ============================================================
// 消息对象池
// ============================================================
class MessagePool
{
public:
    MessagePool(size_t pool_size, size_t payload_size)
        : pool_size_(pool_size),
          payload_size_(payload_size)
    {
        for (size_t i = 0; i < pool_size_; ++i)
        {
            BigData::UniquePtr msg(new BigData());
            msg->payload.resize(payload_size_);

            free_queue_.push_back(std::move(msg));
        }

        RCLCPP_INFO(
            rclcpp::get_logger("message_pool"),
            "Pool initialized: objects=%zu, payload=%.2f MB, total=%.2f MB",
            pool_size_,
            payload_size_ / 1024.0 / 1024.0,
            pool_size_ * payload_size_ / 1024.0 / 1024.0
        );
    }

    // 借出一个消息对象；没有空闲对象时返回 nullptr
    BigData::UniquePtr acquire()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (free_queue_.empty())
        {
            ++failed_;
            return nullptr;
        }

        auto msg = std::move(free_queue_.front());
        free_queue_.pop_front();

        ++acquired_;
        return msg;
    }

    // 归还消息对象
    void release(BigData::UniquePtr msg)
    {
        if (!msg)
            return;

        // 清理业务字段，但保留 payload 已分配的内存
        msg->seq = 0;
        msg->send_time_ns = 0;

        std::lock_guard<std::mutex> lock(mutex_);

        free_queue_.push_back(std::move(msg));
        ++released_;
    }

    size_t available() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return free_queue_.size();
    }

    uint64_t acquired() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return acquired_;
    }

    uint64_t released() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return released_;
    }

    uint64_t failed() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return failed_;
    }

private:
    size_t pool_size_;
    size_t payload_size_;

    mutable std::mutex mutex_;
    std::deque<BigData::UniquePtr> free_queue_;

    uint64_t acquired_{0};
    uint64_t released_{0};
    uint64_t failed_{0};
};

// ============================================================
// Publisher
// ============================================================
class BigDataPublisher : public rclcpp::Node
{
public:
    BigDataPublisher(
        std::shared_ptr<MessagePool> pool,
        double frequency)
        : Node(
            "bigdata_pool_publisher",
            rclcpp::NodeOptions().use_intra_process_comms(true)
          ),
          pool_(std::move(pool)),
          frequency_(frequency)
    {
        // Publisher Timer 独立 Callback Group
        timer_group_ = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive
        );

        publisher_ = create_publisher<BigData>(
            "/bigdata_topic",
            rclcpp::QoS(rclcpp::KeepLast(5)).best_effort()
        );

        auto period =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(1.0 / frequency_)
            );

        timer_ = create_wall_timer(
            period,
            std::bind(&BigDataPublisher::publishData, this),
            timer_group_
        );

        RCLCPP_INFO(
            get_logger(),
            "Publisher started: frequency=%.1f Hz",
            frequency_
        );
    }

private:
    void publishData()
    {
        // 每次 Timer 触发都递增序号。
        // 即使借用失败，下一条成功消息也能体现序号间隙。
        const uint64_t current_seq = seq_++;

        auto msg = pool_->acquire();

        if (!msg)
        {
            const uint64_t failed = pool_->failed();

            if (failed == 1 || failed % 10 == 0)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "Pool exhausted! failed=%lu, available=%zu",
                    static_cast<unsigned long>(failed),
                    pool_->available()
                );
            }

            return;
        }

        msg->seq = current_seq;
        msg->send_time_ns = getTimeNs();

        if (!msg->payload.empty())
        {
            msg->payload[0] =
                static_cast<uint8_t>(current_seq % 256);
        }

        publisher_->publish(std::move(msg));

        if (current_seq % 50 == 0)
        {
            RCLCPP_INFO(
                get_logger(),
                "Published seq=%lu, pool_available=%zu",
                static_cast<unsigned long>(current_seq),
                pool_->available()
            );
        }
    }

private:
    std::shared_ptr<MessagePool> pool_;

    double frequency_;
    uint64_t seq_{0};

    rclcpp::CallbackGroup::SharedPtr timer_group_;
    rclcpp::Publisher<BigData>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// ============================================================
// Subscriber
// ============================================================
class BigDataSubscriber : public rclcpp::Node
{
public:
    explicit BigDataSubscriber(
        std::shared_ptr<MessagePool> pool)
        : Node(
            "bigdata_pool_subscriber",
            rclcpp::NodeOptions().use_intra_process_comms(true)
          ),
          pool_(std::move(pool))
    {
        // Subscriber 使用独立 Callback Group
        sub_group_ = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive
        );

        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = sub_group_;

        subscription_ = create_subscription<BigData>(
            "/bigdata_topic",
            rclcpp::QoS(rclcpp::KeepLast(5)).best_effort(),
            [this](BigData::UniquePtr msg)
            {
                messageCallback(std::move(msg));
            },
            sub_options
        );

        // 统计 Timer 使用默认 Callback Group
        stats_timer_ = create_wall_timer(
            5s,
            std::bind(&BigDataSubscriber::printStatistics, this)
        );

        RCLCPP_INFO(get_logger(), "Subscriber started");
    }

private:
    void messageCallback(BigData::UniquePtr msg)
    {
        if (!msg)
            return;

        // 计算消息从发布到接收的时间差
        const uint64_t now_ns = getTimeNs();

        double latency_ms = 0.0;

        if (now_ns >= msg->send_time_ns)
        {
            latency_ms =
                (now_ns - msg->send_time_ns) / 1000000.0;
        }

        // 模拟耗时业务处理：300ms
        // 注意：必须在归还对象之前处理
        std::this_thread::sleep_for(300ms);

        // 记录统计数据
        {
            std::lock_guard<std::mutex> lock(stats_mutex_);

            ++received_;
            latency_samples_.push_back(latency_ms);

            if (has_last_seq_ && msg->seq > last_seq_ + 1)
            {
                missing_seq_ += msg->seq - last_seq_ - 1;
            }

            last_seq_ = msg->seq;
            has_last_seq_ = true;
        }

        // 处理完成后归还对象池
        pool_->release(std::move(msg));
    }

    void printStatistics()
    {
        uint64_t received = 0;
        uint64_t missing = 0;
        std::vector<double> latencies;

        {
            std::lock_guard<std::mutex> lock(stats_mutex_);

            received = received_;
            missing = missing_seq_;

            latencies.swap(latency_samples_);
            received_ = 0;
        }

        const double elapsed =
            std::chrono::duration<double>(
                std::chrono::steady_clock::now() -
                last_stats_time_
            ).count();

        last_stats_time_ = std::chrono::steady_clock::now();

        const double rate =
            elapsed > 0.0 ? received / elapsed : 0.0;

        double avg = 0.0;
        double p95 = 0.0;
        double max_latency = 0.0;

        if (!latencies.empty())
        {
            for (double value : latencies)
                avg += value;

            avg /= latencies.size();

            std::sort(latencies.begin(), latencies.end());

            const size_t p95_index =
                static_cast<size_t>(
                    std::ceil(latencies.size() * 0.95)
                ) - 1;

            p95 = latencies[p95_index];
            max_latency = latencies.back();
        }

        RCLCPP_INFO(
            get_logger(),
            "\n========== Pool Benchmark ==========\n"
            "received       : %lu\n"
            "receive rate   : %.2f Hz\n"
            "latency avg    : %.3f ms\n"
            "latency p95    : %.3f ms\n"
            "latency max    : %.3f ms\n"
            "missing seq    : %lu\n"
            "pool available : %zu\n"
            "pool acquired  : %lu\n"
            "pool released  : %lu\n"
            "pool failed    : %lu\n"
            "====================================",
            static_cast<unsigned long>(received),
            rate,
            avg,
            p95,
            max_latency,
            static_cast<unsigned long>(missing),
            pool_->available(),
            static_cast<unsigned long>(pool_->acquired()),
            static_cast<unsigned long>(pool_->released()),
            static_cast<unsigned long>(pool_->failed())
        );
    }

private:
    std::shared_ptr<MessagePool> pool_;

    rclcpp::CallbackGroup::SharedPtr sub_group_;

    rclcpp::Subscription<BigData>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr stats_timer_;

    std::mutex stats_mutex_;

    uint64_t received_{0};
    uint64_t missing_seq_{0};

    uint64_t last_seq_{0};
    bool has_last_seq_{false};

    std::vector<double> latency_samples_;

    std::chrono::steady_clock::time_point last_stats_time_{
        std::chrono::steady_clock::now()
    };
};

// ============================================================
// main
// ============================================================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // 参数：
    // 1. 每条消息 payload 大小，单位 MB
    // 2. Publisher 发布频率，单位 Hz
    // 3. 对象池容量

    size_t size_mb = 1;
    double frequency = 10.0;
    size_t pool_size = 5;

    if (argc > 1)
        size_mb = std::strtoull(argv[1], nullptr, 10);

    if (argc > 2)
        frequency = std::atof(argv[2]);

    if (argc > 3)
        pool_size = std::strtoull(argv[3], nullptr, 10);

    if (size_mb == 0 || frequency <= 0.0 || pool_size == 0)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("main"),
            "Invalid arguments"
        );

        rclcpp::shutdown();
        return 1;
    }

    const size_t payload_size =
        size_mb * 1024 * 1024;

    auto pool = std::make_shared<MessagePool>(
        pool_size,
        payload_size
    );

    // 保持 Node 生命周期
    auto publisher = std::make_shared<BigDataPublisher>(
        pool,
        frequency
    );

    auto subscriber = std::make_shared<BigDataSubscriber>(
        pool
    );

    RCLCPP_INFO(
        rclcpp::get_logger("main"),
        "Pool benchmark started: size=%zu MB, frequency=%.1f Hz, pool=%zu",
        size_mb,
        frequency,
        pool_size
    );

    // 两个线程：Publisher 和 Subscriber 可以并发执行
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(),
        2
    );

    executor.add_node(publisher);
    executor.add_node(subscriber);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
