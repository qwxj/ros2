
#include <chrono>
#include <memory>
#include <functional>
#include <thread>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

class TaskIsolationDemo : public rclcpp::Node
{
public:
    TaskIsolationDemo()
        : Node("task_isolation_demo"),
          control_seq_(0),
          sensor_seq_(0)
    {
        // 1. 创建两个不同的 MutuallyExclusive Callback Group
        control_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        sensor_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        // 2. 创建 SubscriptionOptions
        rclcpp::SubscriptionOptions control_options;
        control_options.callback_group = control_group_;

        rclcpp::SubscriptionOptions sensor_options;
        sensor_options.callback_group = sensor_group_;

        // 3. 创建控制指令 Subscriber
        control_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/control_topic",
            10,
            std::bind(
                &TaskIsolationDemo::control_callback,
                this,
                std::placeholders::_1),
            control_options);

        // 4. 创建传感器 Subscriber
        sensor_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/sensor_topic",
            10,
            std::bind(
                &TaskIsolationDemo::sensor_callback,
                this,
                std::placeholders::_1),
            sensor_options);

        RCLCPP_INFO(
            this->get_logger(),
            "Task Isolation Demo started");
    }

private:
    // 控制任务：快速执行
    void control_callback(
        const std_msgs::msg::Int32::SharedPtr msg)
    {
        auto thread_id =
            std::hash<std::thread::id>{}(
                std::this_thread::get_id());

        RCLCPP_INFO(
            this->get_logger(),
            "[控制开始] seq=%d, thread=%zu",
            msg->data,
            thread_id);

        // 模拟快速控制处理
        std::this_thread::sleep_for(5ms);

        RCLCPP_INFO(
            this->get_logger(),
            "[控制结束] seq=%d, thread=%zu",
            msg->data,
            thread_id);
    }

    // 传感器任务：模拟耗时处理
    void sensor_callback(
        const std_msgs::msg::Int32::SharedPtr msg)
    {
        auto thread_id =
            std::hash<std::thread::id>{}(
                std::this_thread::get_id());

        RCLCPP_INFO(
            this->get_logger(),
            "[传感器开始] seq=%d, thread=%zu",
            msg->data,
            thread_id);

        // 模拟传感器数据处理耗时 300ms
        std::this_thread::sleep_for(300ms);

        RCLCPP_INFO(
            this->get_logger(),
            "[传感器结束] seq=%d, thread=%zu",
            msg->data,
            thread_id);
    }

    // Callback Groups
    rclcpp::CallbackGroup::SharedPtr control_group_;
    rclcpp::CallbackGroup::SharedPtr sensor_group_;

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr
        control_sub_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr
        sensor_sub_;

    int control_seq_;
    int sensor_seq_;
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

    auto demo_node = std::make_shared<TaskIsolationDemo>();
    auto control_node = std::make_shared<ControlPublisher>();
    auto sensor_node = std::make_shared<SensorPublisher>();

    // 使用 4 个工作线程
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 4);

    executor.add_node(demo_node);
    executor.add_node(control_node);
    executor.add_node(sensor_node);

    executor.spin();

    rclcpp::shutdown();

    return 0;
}
