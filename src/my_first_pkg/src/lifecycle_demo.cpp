
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

using namespace std::chrono_literals;

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class LifecycleDemo : public rclcpp_lifecycle::LifecycleNode
{
public:
    LifecycleDemo()
        : LifecycleNode("lifecycle_demo")
    {
        RCLCPP_INFO(get_logger(), "生命周期节点已创建");
    }

protected:
    // 1. 配置：创建定时器，但不启动
    CallbackReturn on_configure(
        const rclcpp_lifecycle::State &) override
    {
        timer_ = this->create_wall_timer(
            1s,
            [this]() {
                RCLCPP_INFO(get_logger(), "设备正在运行");
            });

        timer_->cancel();

        RCLCPP_INFO(get_logger(), "配置完成，定时器已创建");
        return CallbackReturn::SUCCESS;
    }

    // 2. 激活：启动定时器
    CallbackReturn on_activate(
        const rclcpp_lifecycle::State &) override
    {
        if (timer_) {
            timer_->reset();
        }

        RCLCPP_INFO(get_logger(), "节点已激活");
        return CallbackReturn::SUCCESS;
    }

    // 3. 停用：停止定时器，但保留资源
    CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &) override
    {
        if (timer_) {
            timer_->cancel();
        }

        RCLCPP_INFO(get_logger(), "节点已停用");
        return CallbackReturn::SUCCESS;
    }

    // 4. 清理：释放定时器
    CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State &) override
    {
        timer_.reset();

        RCLCPP_INFO(get_logger(), "资源已清理");
        return CallbackReturn::SUCCESS;
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<LifecycleDemo>();

    rclcpp::spin(node->get_node_base_interface());

    rclcpp::shutdown();
    return 0;
}
