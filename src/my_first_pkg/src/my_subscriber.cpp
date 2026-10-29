
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <memory>

class MySubscriber : public rclcpp::Node
{
public:
    MySubscriber() : Node("my_subscriber")
    {
        subscription_ =
            this->create_subscription<std_msgs::msg::String>(
                "my_topic",
                10,
                [this](std_msgs::msg::String::ConstSharedPtr msg)
                {
                    if (msg->data.find("4") != std::string::npos)
		    {
			RCLCPP_INFO(
			    this->get_logger(),
			    "匹配到消息: '%s'",
			    msg->data.c_str());
		    }
                });
    }

private:
    rclcpp::Subscription<
        std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<MySubscriber>());

    rclcpp::shutdown();
    return 0;
}

