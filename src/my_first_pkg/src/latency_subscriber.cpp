
// #include <memory>
// #include <chrono>
// #include <functional>

// #include "rclcpp/rclcpp.hpp"
// #include "my_robot_interfaces/msg/latency_data.hpp"

// using namespace std::chrono_literals;

// class LatencySubscriber : public rclcpp::Node
// {
// public:
//     LatencySubscriber()
//         : Node("latency_subscriber")
//     {
//         // 创建 Subscriber
//         subscription_ = this->create_subscription<
//             my_robot_interfaces::msg::LatencyData>(
//                 "/latency_topic",
//                 1,
//                 std::bind(
//                     &LatencySubscriber::callback,
//                     this,
//                     std::placeholders::_1));

//         RCLCPP_INFO(
//             this->get_logger(),
//             "Latency Subscriber started");
//     }

// private:
//     void callback(
//         const my_robot_interfaces::msg::LatencyData::SharedPtr msg)
//     {
//         // 获取当前时间
//         auto now = this->get_clock()->now();

//         // 计算消息延迟
//         double latency_ms =
//             (now - rclcpp::Time(msg->stamp)).seconds() * 1000.0;

//         RCLCPP_INFO(
//             this->get_logger(),
//             "Received seq=%lu, latency=%.3f ms, data=%s",
//             msg->seq,
//             latency_ms,
//             msg->data.c_str());

//         // 模拟耗时业务处理
//         // 后续实验再打开
//         rclcpp::sleep_for(300ms);
//     }

//     rclcpp::Subscription<
//         my_robot_interfaces::msg::LatencyData>::SharedPtr subscription_;
// };

// int main(int argc, char ** argv)
// {
//     rclcpp::init(argc, argv);

//     auto node = std::make_shared<LatencySubscriber>();

//     rclcpp::spin(node);

//     rclcpp::shutdown();

//     return 0;
// }



#include <memory>
#include <chrono>
#include <functional>
#include <thread>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/latency_data.hpp"

using namespace std::chrono_literals;

class LatencySubscriber : public rclcpp::Node
{
public:
    LatencySubscriber()
        : Node("latency_subscriber")
    {
        // 通过 ROS 参数选择 Callback Group 类型
        this->declare_parameter<std::string>(
            "callback_mode", "mutually");

        std::string mode =
            this->get_parameter("callback_mode").as_string();

        rclcpp::CallbackGroupType group_type;

        if (mode == "reentrant") {
            group_type = rclcpp::CallbackGroupType::Reentrant;
            RCLCPP_INFO(
                this->get_logger(),
                "Callback Group: Reentrant");
        } else {
            group_type = rclcpp::CallbackGroupType::MutuallyExclusive;
            RCLCPP_INFO(
                this->get_logger(),
                "Callback Group: MutuallyExclusive");
        }

        // 创建 Callback Group
        callback_group_ =
            this->create_callback_group(group_type);

        // 将 Subscriber 放入指定的 Callback Group
        rclcpp::SubscriptionOptions options;
        options.callback_group = callback_group_;

        subscription_ = this->create_subscription<
            my_robot_interfaces::msg::LatencyData>(
                "/latency_topic",
                rclcpp::QoS(rclcpp::KeepLast(10)),
                std::bind(
                    &LatencySubscriber::callback,
                    this,
                    std::placeholders::_1),
                options);

        RCLCPP_INFO(
            this->get_logger(),
            "Latency Subscriber started");
    }

private:
    void callback(
        const my_robot_interfaces::msg::LatencyData::SharedPtr msg)
    {
        auto now = this->get_clock()->now();

        double latency_ms =
            (now - rclcpp::Time(msg->stamp)).seconds() * 1000.0;

        auto thread_id =
            std::hash<std::thread::id>{}(
                std::this_thread::get_id());

        RCLCPP_INFO(
            this->get_logger(),
            "[开始] seq=%lu, latency=%.3f ms, thread=%zu",
            msg->seq,
            latency_ms,
            thread_id);

        // 模拟业务处理耗时 300ms
        rclcpp::sleep_for(300ms);

        RCLCPP_INFO(
            this->get_logger(),
            "[结束] seq=%lu, thread=%zu",
            msg->seq,
            thread_id);
    }

    // 必须保存 Callback Group 的 shared_ptr
    rclcpp::CallbackGroup::SharedPtr callback_group_;

    rclcpp::Subscription<
        my_robot_interfaces::msg::LatencyData>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<LatencySubscriber>();

    // 创建 4 个工作线程
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 4);

    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
