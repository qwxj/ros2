
#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/get_device_status.hpp"

#include <memory>
#include <string>

using GetDeviceStatus =
    my_robot_interfaces::srv::GetDeviceStatus;

class DeviceStatusServer : public rclcpp::Node
{
public:
    DeviceStatusServer()
        : Node("device_status_server")
    {
        service_ = this->create_service<GetDeviceStatus>(
            "get_device_status",
            [this](
                const std::shared_ptr<GetDeviceStatus::Request> request,
                std::shared_ptr<GetDeviceStatus::Response> response)
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "收到设备查询请求：%s",
                    request->device_name.c_str());

                if (request->device_name == "lidar") {
                    response->success = true;
                    response->message = "激光雷达运行正常";
                    response->temperature = 42.5;
                }
                else if (request->device_name == "camera") {
                    response->success = true;
                    response->message = "摄像头运行正常";
                    response->temperature = 36.8;
                }
                else {
                    response->success = false;
                    response->message = "未找到该设备";
                    response->temperature = 0.0;
                }

                RCLCPP_INFO(
                    this->get_logger(),
                    "返回结果：%s，温度：%.1f",
                    response->message.c_str(),
                    response->temperature);
            });

        RCLCPP_INFO(
            this->get_logger(),
            "设备状态服务已启动");
    }

private:
    rclcpp::Service<GetDeviceStatus>::SharedPtr service_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DeviceStatusServer>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}

