
#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/get_device_status.hpp"

#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;

using GetDeviceStatus =
    my_robot_interfaces::srv::GetDeviceStatus;

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared(
        "device_status_client");

    // 创建服务客户端
    auto client = node->create_client<GetDeviceStatus>(
        "get_device_status");

    // 等待服务端启动
    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(
                node->get_logger(),
                "等待服务时收到退出请求");
            rclcpp::shutdown();
            return 1;
        }

        RCLCPP_INFO(
            node->get_logger(),
            "等待设备状态服务启动...");
    }

    // 构造请求
    auto request =
        std::make_shared<GetDeviceStatus::Request>();

    request->device_name = "camera";

    RCLCPP_INFO(
        node->get_logger(),
        "正在查询设备：%s",
        request->device_name.c_str());

    // 异步发送请求
    auto future = client->async_send_request(request);

    // 等待响应
    auto result = rclcpp::spin_until_future_complete(
        node, future, 5s);

    if (result == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future.get();

        RCLCPP_INFO(
            node->get_logger(),
            "查询结果：%s",
            response->success ? "成功" : "失败");

        RCLCPP_INFO(
            node->get_logger(),
            "设备信息：%s",
            response->message.c_str());

        RCLCPP_INFO(
            node->get_logger(),
            "设备温度：%.1f",
            response->temperature);
    }
    else {
        RCLCPP_ERROR(
            node->get_logger(),
            "请求失败或等待响应超时");
    }

    rclcpp::shutdown();
    return 0;
}

