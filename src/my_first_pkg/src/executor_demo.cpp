
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <memory>
#include <thread>

using namespace std::chrono_literals;

class ExecutorDemo : public rclcpp::Node
{
public:
    ExecutorDemo()
        : Node("executor_demo")
    {
    
    group_a_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

	group_b_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
        
    // 定时器 A：每秒触发一次，但回调本身耗时 2 秒
    timer_a_ = this->create_wall_timer(
		1s,
		[this]() {
		    RCLCPP_INFO(this->get_logger(), "定时器 A 开始执行");
		    std::this_thread::sleep_for(2s);
		    RCLCPP_INFO(this->get_logger(), "定时器 A 执行结束");
		},
		group_a_);

   timer_b_ = this->create_wall_timer(
		1s,
		[this]() {
		    RCLCPP_INFO(this->get_logger(), "定时器 B 执行");
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

    auto node = std::make_shared<ExecutorDemo>();

    rclcpp::executors::MultiThreadedExecutor executor;

    // 将节点加入执行器
    executor.add_node(node);

    // 开始执行回调
    executor.spin();

    rclcpp::shutdown();
    return 0;
}

