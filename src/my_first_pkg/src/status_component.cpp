#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

class StatusComponent : public rclcpp::Node
{
public:
    explicit StatusComponent(
        const rclcpp::NodeOptions & options)
        : Node("status_component", options)
    {
        timer_ = this->create_wall_timer(
            2s,
            [this]() {
                RCLCPP_INFO(
                    this->get_logger(),
                    "组件正在运行");
            });

        RCLCPP_INFO(
            this->get_logger(),
            "StatusComponent 已创建");
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(StatusComponent)