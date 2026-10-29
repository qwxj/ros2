
#include <memory>
#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class OdomBroadcaster : public rclcpp::Node
{
public:
    OdomBroadcaster()
        : Node("tf2_odom_broadcaster_demo"),
          x_(0.0)
    {

        this->declare_parameter<double>("speed", 0.1);
        this->get_parameter("speed", speed_);
        // 创建 TF2 动态变换发布器
        broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // 每隔 100 毫秒发布一次坐标变换
        timer_ = create_wall_timer(
            100ms,
            std::bind(&OdomBroadcaster::publish_transform, this)
        );
    }

private:
    void publish_transform()
    {
        geometry_msgs::msg::TransformStamped transform;

        // 时间戳
        transform.header.stamp = this->get_clock()->now();

        // 父坐标系和子坐标系
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_link";

        // 模拟机器人沿 X 轴匀速前进
        // x_ += 0.01;
        x_ += speed_ * 0.1;

        transform.transform.translation.x = x_;
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = 0.0;

        // 单位四元数：表示没有旋转
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.0;
        transform.transform.rotation.w = 1.0;

        // 发布动态坐标变换
        broadcaster_->sendTransform(transform);

        RCLCPP_INFO(
            this->get_logger(),
            "odom -> base_link: x = %.2f",
            x_
        );
    }

    double x_;

    double speed_{0.1};

    std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<OdomBroadcaster>());

    rclcpp::shutdown();
    return 0;
}
