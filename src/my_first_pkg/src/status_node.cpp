#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class StatusNode : public rclcpp::Node
{
public:
    StatusNode()
        : Node("status_node")
    {
        // 声明参数，默认每 2000 毫秒输出一次
        this->declare_parameter<int>("interval_ms", 2000);

        int interval_ms =
            this->get_parameter("interval_ms").as_int();

        RCLCPP_INFO(
            this->get_logger(),
            "日志间隔: %d ms",
            interval_ms);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(interval_ms),
            [this]() {
                RCLCPP_INFO(
                    this->get_logger(),
                    "机器人状态正常");
            });
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StatusNode>());
    rclcpp::shutdown();
    return 0;
}