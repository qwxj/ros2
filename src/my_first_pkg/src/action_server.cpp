
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "example_interfaces/action/fibonacci.hpp"

#include <memory>
#include <thread>

using Fibonacci = example_interfaces::action::Fibonacci;
using GoalHandleFibonacci =
    rclcpp_action::ServerGoalHandle<Fibonacci>;

class FibonacciActionServer : public rclcpp::Node
{
public:
    FibonacciActionServer()
        : Node("fibonacci_action_server")
    {
        // 创建 Action Server
        action_server_ = rclcpp_action::create_server<Fibonacci>(
            this,
            "fibonacci",

            // 1. 收到 Goal 时，决定是否接受
            [this](
                const rclcpp_action::GoalUUID & uuid,
                std::shared_ptr<const Fibonacci::Goal> goal)
            {
                return handle_goal(uuid, goal);
            },

            // 2. 收到取消请求时的处理
            [this](
                const std::shared_ptr<GoalHandleFibonacci> goal_handle)
            {
                return handle_cancel(goal_handle);
            },

            // 3. 接受 Goal 后，开始执行任务
            [this](
                const std::shared_ptr<GoalHandleFibonacci> goal_handle)
            {
                handle_accepted(goal_handle);
            });

        RCLCPP_INFO(
            this->get_logger(),
            "Fibonacci Action Server 已启动");
    }

private:
    // 判断是否接受 Goal
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Fibonacci::Goal> goal)
    {
        (void)uuid;

        RCLCPP_INFO(
            this->get_logger(),
            "收到目标 order = %d",
            goal->order);

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // 处理取消请求
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
        (void)goal_handle;

        RCLCPP_INFO(
            this->get_logger(),
            "收到取消请求");

        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // 接受目标后，启动执行任务
    void handle_accepted(
        const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
        std::thread(
            [this, goal_handle]()
            {
                execute(goal_handle);
            }).detach();
    }

    // 实际执行斐波那契数列计算
    void execute(
        const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "开始执行任务");

        auto result = std::make_shared<Fibonacci::Result>();
        auto feedback = std::make_shared<Fibonacci::Feedback>();

        // 数列初始值
        result->sequence.push_back(0);

        if (goal_handle->get_goal()->order > 0)
        {
            result->sequence.push_back(1);
        }

        for (int i = 2;
             i <= goal_handle->get_goal()->order;
             ++i)
        {
            // 如果收到取消请求，结束任务
            if (goal_handle->is_canceling())
            {
                goal_handle->canceled(result);
                RCLCPP_INFO(
                    this->get_logger(),
                    "任务已取消");
                return;
            }

            const auto & seq = result->sequence;

            result->sequence.push_back(
                seq[seq.size() - 1] +
                seq[seq.size() - 2]);

            // 发布当前进度
            feedback->sequence = result->sequence;
            goal_handle->publish_feedback(feedback);

            RCLCPP_INFO(
                this->get_logger(),
                "已计算到第 %d 项",
                i);

            std::this_thread::sleep_for(
                std::chrono::seconds(1));
        }

        // 通知客户端：任务执行成功
        goal_handle->succeed(result);

        RCLCPP_INFO(
            this->get_logger(),
            "任务执行成功");
    }

    rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<FibonacciActionServer>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}

