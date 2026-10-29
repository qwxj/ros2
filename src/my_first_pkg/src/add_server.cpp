
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <memory>

using AddTwoInts = example_interfaces::srv::AddTwoInts;

class AddServer : public rclcpp::Node
{
public:
    AddServer() : Node("add_server")
    {
        service_ = this->create_service<AddTwoInts>(
            "add_two_ints",
            [this](
                const std::shared_ptr<AddTwoInts::Request> request,
                std::shared_ptr<AddTwoInts::Response> response)
            {
                response->sum = request->a + request->b;

                RCLCPP_INFO(
                    this->get_logger(),
                    "收到请求: %ld + %ld = %ld",
                    static_cast<long>(request->a),
                    static_cast<long>(request->b),
                    static_cast<long>(response->sum));
            });

        RCLCPP_INFO(
            this->get_logger(),
            "加法服务已启动，等待请求...");
    }

private:
    rclcpp::Service<AddTwoInts>::SharedPtr service_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AddServer>());
    rclcpp::shutdown();
    return 0;
}

