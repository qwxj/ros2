
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class TF2ListenerDemo : public rclcpp::Node
{
public:
    TF2ListenerDemo()
        : Node("tf2_listener_demo")
    {
        // 创建 TF2 缓冲区，用于保存接收到的坐标变换
        buffer_ = std::make_shared<tf2_ros::Buffer>(
            this->get_clock());

        // 创建监听器，自动接收 /tf 和 /tf_static 中的变换
        listener_ = std::make_shared<tf2_ros::TransformListener>(
            *buffer_);

        // 每秒查询一次 base_link 与 laser 之间的变换
        timer_ = this->create_wall_timer(
            1s,
            [this]() {
                query_transform();
            });

        RCLCPP_INFO(
            this->get_logger(),
            "TF2 查询节点已启动");
    }

private:
    void query_transform()
    {
        try
        {
            // 查询 laser 坐标系相对于 odom 的变换
            auto transform = buffer_->lookupTransform(
                "odom",
                "laser",
                tf2::TimePointZero);

            const auto & t = transform.transform.translation;
            const auto & r = transform.transform.rotation;

            RCLCPP_INFO(
                this->get_logger(),
                "laser 相对于 odom: "
                "x=%.2f, y=%.2f, z=%.2f",
                t.x, t.y, t.z);

            RCLCPP_INFO(
                this->get_logger(),
                "旋转四元数: x=%.2f, y=%.2f, z=%.2f, w=%.2f",
                r.x, r.y, r.z, r.w);
        }
        catch (const tf2::TransformException & ex)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "查询坐标变换失败: %s",
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

    auto node = std::make_shared<TF2ListenerDemo>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}

