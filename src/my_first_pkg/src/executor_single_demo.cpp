
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class ExecutorSingleDemo : public rclcpp::Node
{
public:
    ExecutorSingleDemo()
        : Node("executor_single_demo")
    {
        timer_a_ = this->create_wall_timer(
            1s,
            [this]() {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Timer A 开始，准备睡眠 2 秒");

                rclcpp::sleep_for(2s);

                RCLCPP_INFO(
                    this->get_logger(),
                    "Timer A 结束");
            });

        timer_b_ = this->create_wall_timer(
            500ms,
            [this]() {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Timer B 执行");
            });
    }

private:
    rclcpp::TimerBase::SharedPtr timer_a_;
    rclcpp::TimerBase::SharedPtr timer_b_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ExecutorSingleDemo>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
