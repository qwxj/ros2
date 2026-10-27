
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/exceptions.h"
#include "tf2/time.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class TF2PointTransformDemo : public rclcpp::Node
{
public:
    TF2PointTransformDemo()
        : Node("tf2_point_transform_demo")
    {
        // 创建坐标变换缓存
        buffer_ = std::make_shared<tf2_ros::Buffer>(
            this->get_clock());

        // 自动监听其他节点发布的 TF 变换
        listener_ = std::make_shared<tf2_ros::TransformListener>(
            *buffer_);

        // 每秒转换一次障碍物坐标
        timer_ = this->create_wall_timer(
            1s,
            [this]() {
                transform_point();
            });

        RCLCPP_INFO(
            this->get_logger(),
            "障碍物坐标转换节点已启动");
    }

private:
    void transform_point()
    {
        try
        {
            // 假设雷达检测到了一个障碍物
            // 障碍物坐标位于 laser 坐标系
            geometry_msgs::msg::PointStamped point_laser;

            point_laser.header.frame_id = "laser";
            point_laser.header.stamp = this->get_clock()->now();

            point_laser.point.x = 2.0;
            point_laser.point.y = 0.0;
            point_laser.point.z = 0.0;

            // 查询 laser 到 base_link 的坐标变换
            auto transform = buffer_->lookupTransform(
                "base_link",
                "laser",
                tf2::TimePointZero);

            // 将障碍物坐标转换到 base_link 坐标系
            geometry_msgs::msg::PointStamped point_base;

            tf2::doTransform(
                point_laser,
                point_base,
                transform);

            RCLCPP_INFO(
                this->get_logger(),
                "雷达坐标: (%.2f, %.2f, %.2f)",
                point_laser.point.x,
                point_laser.point.y,
                point_laser.point.z);

            RCLCPP_INFO(
                this->get_logger(),
                "机器人坐标: (%.2f, %.2f, %.2f)",
                point_base.point.x,
                point_base.point.y,
                point_base.point.z);
        }
        catch (const tf2::TransformException & ex)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "坐标转换失败: %s",
                ex.what());
        }
    }

    std::shared_ptr<tf2_ros::Buffer> buffer_;
    std::shared_ptr<tf2_ros::TransformListener> listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TF2PointTransformDemo>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}

