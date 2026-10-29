
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "example_interfaces/action/fibonacci.hpp"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;
using Fibonacci = example_interfaces::action::Fibonacci;
using GoalHandleFibonacci =
    rclcpp_action::ClientGoalHandle<Fibonacci>;

class FibonacciActionClient : public rclcpp::Node
{
public:
    FibonacciActionClient()
        : Node("fibonacci_action_client")
    {
        // 创建 Action Client，连接名为 fibonacci 的 Action Server
        client_ = rclcpp_action::create_client<Fibonacci>(
            this, "fibonacci");
    }

    void send_goal(int order)
    {
        // 等待服务端上线
        if (!client_->wait_for_action_server(5s))
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "等待 Action Server 超时");
            rclcpp::shutdown();
            return;
        }

        // 创建目标请求
        auto goal_msg = Fibonacci::Goal();
        goal_msg.order = order;

        RCLCPP_INFO(
            this->get_logger(),
            "发送目标：order = %d",
            order);

        // 配置目标接受、反馈和最终结果的回调
        auto options =
            rclcpp_action::Client<Fibonacci>::SendGoalOptions();

        // 收到服务端的执行反馈时调用
        options.feedback_callback =
            [this](
                GoalHandleFibonacci::SharedPtr,
                const std::shared_ptr<const Fibonacci::Feedback> feedback)
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "收到 Feedback，当前数列长度：%zu",
                    feedback->sequence.size());
            };

        // 收到最终执行结果时调用
        options.result_callback =
            [this](
                const GoalHandleFibonacci::WrappedResult & result)
            {
                if (result.code ==
                    rclcpp_action::ResultCode::SUCCEEDED)
                {
                    std::string sequence;

                    for (const auto value : result.result->sequence)
                    {
                        sequence += std::to_string(value) + " ";
                    }

                    RCLCPP_INFO(
                        this->get_logger(),
                        "任务成功，最终数列：%s",
                        sequence.c_str());
                }
                else
                {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "任务未成功完成");
                }

                rclcpp::shutdown();
            };

        // 发送目标，之后由 ROS 2 事件循环处理反馈和结果
        client_->async_send_goal(goal_msg, options);
    }

private:
    rclcpp_action::Client<Fibonacci>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<FibonacciActionClient>();

    node->send_goal(5);

    rclcpp::spin(node);

    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }

    return 0;
}

