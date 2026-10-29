
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class ParameterDemo : public rclcpp::Node
{
public:
    ParameterDemo()
        : Node("parameter_demo")
    {
        // 1. 声明参数 speed，默认值为 1.0
        this->declare_parameter<double>("speed", 1.0);

        // 2. 创建定时器，每秒执行一次回调
        timer_ = this->create_wall_timer(
            1s,
            [this]()
            {
                // 3. 每次执行时重新读取参数
                double speed =
                    this->get_parameter("speed").as_double();

                RCLCPP_INFO(
                    this->get_logger(),
                    "当前运行速度：%.2f",
                    speed);
            });
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ParameterDemo>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}

