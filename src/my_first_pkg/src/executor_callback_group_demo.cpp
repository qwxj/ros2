#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class ExecutorGroupDemo : public rclcpp::Node
{
public:
    ExecutorGroupDemo()
        : Node("executor_group_demo")
    {
        // 创建两个可重入回调组
        group_a_ = create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);

        group_b_ = create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);

        // Timer A 放入 group_a
        timer_a_ = create_wall_timer(
            1000ms,
            [this]()
            {
                RCLCPP_INFO(
                    get_logger(),
                    "A 开始，线程ID=%zu",
                    std::hash<std::thread::id>{}(
                        std::this_thread::get_id()));

                std::this_thread::sleep_for(2s);

                RCLCPP_INFO(get_logger(), "A 结束");
            },
            group_a_);

        // Timer B 放入 group_b
        timer_b_ = create_wall_timer(
            1000ms,
            [this]()
            {
                RCLCPP_INFO(
                    get_logger(),
                    "B 执行，线程ID=%zu",
                    std::hash<std::thread::id>{}(
                        std::this_thread::get_id()));
            },
            group_b_);
    }

private:
    rclcpp::CallbackGroup::SharedPtr group_a_;
    rclcpp::CallbackGroup::SharedPtr group_b_;

    rclcpp::TimerBase::SharedPtr timer_a_;
    rclcpp::TimerBase::SharedPtr timer_b_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ExecutorGroupDemo>();

    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(),
        2);

    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}