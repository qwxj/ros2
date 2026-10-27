#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/device_status.hpp"

#include <memory>
#include <functional>

using DeviceStatus = my_robot_interfaces::msg::DeviceStatus;

class DeviceStatusSubscriber : public rclcpp::Node
{
public:
    DeviceStatusSubscriber()
        : Node("device_status_subscriber")
    {
        subscription_ = this->create_subscription<DeviceStatus>(
            "device_status",
            10,
            std::bind(
                &DeviceStatusSubscriber::status_callback,
                this,
                std::placeholders::_1));

        RCLCPP_INFO(
            this->get_logger(),
            "设备状态订阅者已启动");
    }

private:
    void status_callback(const DeviceStatus::SharedPtr msg)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "收到设备状态：设备=%s，在线=%s，温度=%.1f",
            msg->device_name.c_str(),
            msg->online ? "是" : "否",
            msg->temperature);
    }

    rclcpp::Subscription<DeviceStatus>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DeviceStatusSubscriber>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
