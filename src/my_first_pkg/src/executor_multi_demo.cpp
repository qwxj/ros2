
#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class ExecutorMultiDemo : public rclcpp::Node
{
public:
    ExecutorMultiDemo()
        : Node("executor_multi_demo")
    {

        group_a_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        group_b_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        timer_a_ = this->create_wall_timer(
            1s,
            [this]() {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Timer A 开始，线程 ID: %zu",
                    std::hash<std::thread::id>{}(std::this_thread::get_id()));

                rclcpp::sleep_for(2s);

                RCLCPP_INFO(
                    this->get_logger(),
                    "Timer A 结束，线程 ID: %zu",
                    std::hash<std::thread::id>{}(std::this_thread::get_id()));
            }, group_a_);

        timer_b_ = this->create_wall_timer(
            500ms,
            [this]() {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Timer B 执行，线程 ID: %zu",
                    std::hash<std::thread::id>{}(std::this_thread::get_id()));
            }, group_b_);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_a_;
    rclcpp::TimerBase::SharedPtr timer_b_;

    rclcpp::CallbackGroup::SharedPtr group_a_;
    rclcpp::CallbackGroup::SharedPtr group_b_;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ExecutorMultiDemo>();

    // 改成多线程执行器
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
