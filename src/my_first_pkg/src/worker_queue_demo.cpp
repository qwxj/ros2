
#include <chrono>
#include <memory>
#include <functional>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

class WorkerQueueDemo : public rclcpp::Node
{
public:
    WorkerQueueDemo()
        : Node("worker_queue_demo"),
          dropped_count_(0),
          stopping_(false)
    {
        // 控制任务 Callback Group
        control_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        // 传感器接收 Callback Group
        sensor_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions control_options;
        control_options.callback_group = control_group_;

        rclcpp::SubscriptionOptions sensor_options;
        sensor_options.callback_group = sensor_group_;

        control_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/control_topic",
            10,
            std::bind(
                &WorkerQueueDemo::control_callback,
                this,
                std::placeholders::_1),
            control_options);

        sensor_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/sensor_topic",
            10,
            std::bind(
                &WorkerQueueDemo::sensor_callback,
                this,
                std::placeholders::_1),
            sensor_options);

        // 启动独立 Worker 线程
        worker_thread_ = std::thread(
            &WorkerQueueDemo::worker_loop,
            this);

        RCLCPP_INFO(
            this->get_logger(),
            "Worker Queue Demo started");
    }

    ~WorkerQueueDemo()
    {
        // 通知 Worker 线程退出
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            stopping_ = true;
        }

        queue_cv_.notify_one();

        // 等待 Worker 线程结束
        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }
    }

private:
    // 控制回调：快速执行
    void control_callback(
        const std_msgs::msg::Int32::SharedPtr msg)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "[控制回调] seq=%d",
            msg->data);
    }

    // 传感器回调：只负责接收并入队
    void sensor_callback(
        const std_msgs::msg::Int32::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);

            // 队列满了，丢弃最旧的数据
            if (sensor_queue_.size() >= 10) {
                sensor_queue_.pop_front();
                ++dropped_count_;
            }

            sensor_queue_.push_back(msg->data);

            RCLCPP_INFO(
                this->get_logger(),
                "[传感器入队] seq=%d, queue_size=%zu, dropped=%zu",
                msg->data,
                sensor_queue_.size(),
                dropped_count_);
        }

        // 通知 Worker 有新数据
        queue_cv_.notify_one();
    }

    // 独立 Worker 线程
    void worker_loop()
    {
        while (true) {
            int data = 0;

            {
                std::unique_lock<std::mutex> lock(queue_mutex_);

                // 没有数据时等待，不占用 CPU 忙等
                queue_cv_.wait(lock, [this]() {
                    return stopping_ || !sensor_queue_.empty();
                });

                if (stopping_) {
                    break;
                }

                // 取出最旧的一条待处理数据
                data = sensor_queue_.front();
                sensor_queue_.pop_front();
            }

            // 注意：锁已经释放，现在执行耗时业务
            RCLCPP_INFO(
                this->get_logger(),
                "[Worker 开始处理] seq=%d, thread=%zu",
                data,
                std::hash<std::thread::id>{}(
                    std::this_thread::get_id()));

            std::this_thread::sleep_for(300ms);

            RCLCPP_INFO(
                this->get_logger(),
                "[Worker 处理完成] seq=%d",
                data);
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Worker thread stopped");
    }

    rclcpp::CallbackGroup::SharedPtr control_group_;
    rclcpp::CallbackGroup::SharedPtr sensor_group_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr control_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sensor_sub_;

    // 线程安全队列
    std::deque<int> sensor_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;

    // Worker 线程
    std::thread worker_thread_;

    // 队列统计与退出状态
    std::size_t dropped_count_;
    bool stopping_;
};

// 控制指令 Publisher
class ControlPublisher : public rclcpp::Node
{
public:
    ControlPublisher()
        : Node("control_publisher"),
          seq_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::Int32>(
            "/control_topic", 10);

        timer_ = this->create_wall_timer(
            100ms,
            std::bind(&ControlPublisher::publish_data, this));
    }

private:
    void publish_data()
    {
        std_msgs::msg::Int32 msg;
        msg.data = seq_++;
        publisher_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int seq_;
};

// 传感器 Publisher
class SensorPublisher : public rclcpp::Node
{
public:
    SensorPublisher()
        : Node("sensor_publisher"),
          seq_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::Int32>(
            "/sensor_topic", 10);

        timer_ = this->create_wall_timer(
            100ms,
            std::bind(&SensorPublisher::publish_data, this));
    }

private:
    void publish_data()
    {
        std_msgs::msg::Int32 msg;
        msg.data = seq_++;
        publisher_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int seq_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto worker_node = std::make_shared<WorkerQueueDemo>();
    auto control_node = std::make_shared<ControlPublisher>();
    auto sensor_node = std::make_shared<SensorPublisher>();

    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 4);

    executor.add_node(worker_node);
    executor.add_node(control_node);
    executor.add_node(sensor_node);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
