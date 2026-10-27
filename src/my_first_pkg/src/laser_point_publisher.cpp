#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

using namespace std::chrono_literals;

class LaserPointPublisher : public rclcpp::Node
{
public:
    LaserPointPublisher()
        : Node("laser_point_publisher")
    {
        publisher_ =
            this->create_publisher<
                geometry_msgs::msg::PointStamped>(
                    "/laser_point", 10);

        timer_ = this->create_wall_timer(
            1s,
            std::bind(
                &LaserPointPublisher::publish_point,
                this));
    }

private:
    void publish_point()
    {
        geometry_msgs::msg::PointStamped point;

        point.header.stamp = this->now();
        point.header.frame_id = "laser";

        point.point.x = 2.0;
        point.point.y = 0.0;
        point.point.z = 0.0;

        publisher_->publish(point);

        RCLCPP_INFO(
            this->get_logger(),
            "发布雷达点: (%.2f, %.2f, %.2f), frame=%s",
            point.point.x,
            point.point.y,
            point.point.z,
            point.header.frame_id.c_str());
    }

    rclcpp::Publisher<
        geometry_msgs::msg::PointStamped>::SharedPtr publisher_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<LaserPointPublisher>());

    rclcpp::shutdown();
    return 0;
}