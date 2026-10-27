
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;
using AddTwoInts = example_interfaces::srv::AddTwoInts;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("add_client");
    auto client = node->create_client<AddTwoInts>("add_two_ints");

    while (!client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(node->get_logger(), "等待服务时被中断");
            rclcpp::shutdown();
            return 1;
        }

        RCLCPP_INFO(node->get_logger(), "等待加法服务启动...");
    }

    auto request = std::make_shared<AddTwoInts::Request>();
    request->a = 10;
    request->b = 20;

    auto future = client->async_send_request(request);

    auto result = rclcpp::spin_until_future_complete(node, future);

    if (result == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(
            node->get_logger(),
            "计算结果: %ld",
            static_cast<long>(future.get()->sum));
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "请求失败");
    }

    rclcpp::shutdown();
    return result == rclcpp::FutureReturnCode::SUCCESS ? 0 : 1;
}

