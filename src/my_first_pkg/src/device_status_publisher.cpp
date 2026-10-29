#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/device_status.hpp"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;
using DeviceStatus = my_robot_interfaces::msg::DeviceStatus;

class DeviceStatusPublisher : public rclcpp::Node
{
public:
    DeviceStatusPublisher()
        : Node("device_status_publisher")
    {
        publisher_ = this->create_publisher<DeviceStatus>(
            "device_status", 10);

        timer_ = this->create_wall_timer(
            1s,
            std::bind(
                &DeviceStatusPublisher::publish_status,
                this));

        RCLCPP_INFO(
            this->get_logger(),
            "设备状态发布者已启动");
    }

private:
    void publish_status()
    {
        DeviceStatus msg;

        msg.device_name = "lidar";
        msg.online = true;
        msg.temperature = 42.5;

        publisher_->publish(msg);

        RCLCPP_INFO(
            this->get_logger(),
            "设备：%s，在线：%s，温度：%.1f",
            msg.device_name.c_str(),
            msg.online ? "是" : "否",
            msg.temperature);
    }

    rclcpp::Publisher<DeviceStatus>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DeviceStatusPublisher>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
