
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class LaserPointTransformer : public rclcpp::Node
{
public:
    LaserPointTransformer()
        : Node("laser_point_transformer")
    {
        // 1. 创建 TF2 缓存
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(
            this->get_clock());

        // 2. 创建 TF2 监听器
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(
                *tf_buffer_);

        // 3. 订阅雷达点话题
        subscription_ =
            this->create_subscription<
                geometry_msgs::msg::PointStamped>(
                "/laser_point",
                10,
                std::bind(
                    &LaserPointTransformer::point_callback,
                    this,
                    std::placeholders::_1));

        RCLCPP_INFO(
            this->get_logger(),
            "坐标转换节点已启动，等待 /laser_point");
    }

private:
    void point_callback(
        const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        try
        {
            // 将雷达坐标系下的点转换到机器人坐标系
            geometry_msgs::msg::PointStamped point_base;
            
            auto timeout = rclcpp::Duration::from_seconds(0.2);

            if (!tf_buffer_->canTransform(
                    "odom",
                    msg->header.frame_id,
                    rclcpp::Time(msg->header.stamp),
                    timeout)) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "等待 TF 超时，跳过这条消息");
                return;
            }

            auto transform = tf_buffer_->lookupTransform(
                "odom",
                msg->header.frame_id,
                rclcpp::Time(msg->header.stamp));

            // 使用最新变换转换点
            tf2::doTransform(
                *msg,
                point_base,
                transform);

            // tf_buffer_->transform(
            //     *msg,
            //     point_base,
            //     "base_link");

            RCLCPP_INFO(
                this->get_logger(),
                "转换前 laser: (%.2f, %.2f, %.2f)",
                msg->point.x,
                msg->point.y,
                msg->point.z);

            RCLCPP_INFO(
                this->get_logger(),
                "转换后 odom: (%.2f, %.2f, %.2f)",
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

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<
        geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<LaserPointTransformer>());

    rclcpp::shutdown();
    return 0;
}
