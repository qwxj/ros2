// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/transform_stamped.hpp"
// #include "tf2_ros/transform_broadcaster.h"

// #include <chrono>
// #include <memory>
// #include <functional>

// using namespace std::chrono_literals;

// class TF2BroadcasterDemo : public rclcpp::Node
// {
// public:
//     TF2BroadcasterDemo()
//         : Node("tf2_broadcaster_demo")
//     {
//         // 创建 TF2 动态坐标变换广播器
//         broadcaster_ =
//             std::make_shared<tf2_ros::TransformBroadcaster>(this);

//         // 每隔 100 毫秒发布一次坐标变换
//         timer_ = this->create_wall_timer(
//             100ms,
//             std::bind(
//                 &TF2BroadcasterDemo::publish_transform,
//                 this));

//         RCLCPP_INFO(
//             this->get_logger(),
//             "TF2 动态坐标变换广播器已启动");
//     }

// private:
//     void publish_transform()
//     {
//         // 定义一个带时间戳的坐标变换消息
//         geometry_msgs::msg::TransformStamped transform;

//         // 记录当前时间
//         transform.header.stamp = this->get_clock()->now();

//         // 父坐标系：机器人本体
//         transform.header.frame_id = "base_link";

//         // 子坐标系：激光雷达
//         transform.child_frame_id = "laser";

//         // laser 原点在 base_link 坐标系中的位置，单位为米
//         transform.transform.translation.x = 1.0;
//         transform.transform.translation.y = 0.5;
//         transform.transform.translation.z = 0.2;

//         // 设置旋转四元数
//         // x、y、z 为 0，w 为 1，表示没有旋转
//         transform.transform.rotation.x = 0.0;
//         transform.transform.rotation.y = 0.0;
//         // transform.transform.rotation.z = 0.0;
//         // transform.transform.rotation.w = 1.0;
//         transform.transform.rotation.z = 0.70710678;
// 	transform.transform.rotation.w = 0.70710678;

//         // 发布坐标变换
//         broadcaster_->sendTransform(transform);
//     }

//     // 保存广播器对象
//     std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

//     // 定时器对象，避免定时器被销毁
//     rclcpp::TimerBase::SharedPtr timer_;
// };

// int main(int argc, char ** argv)
// {
//     // 初始化 ROS 2
//     rclcpp::init(argc, argv);

//     // 创建广播器节点
//     auto node = std::make_shared<TF2BroadcasterDemo>();

//     // 持续处理节点事件和定时器回调
//     rclcpp::spin(node);

//     // 关闭 ROS 2
//     rclcpp::shutdown();

//     return 0;
// }




#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

class LaserStaticBroadcaster : public rclcpp::Node
{
public:
    LaserStaticBroadcaster()
        : Node("tf2_broadcaster_demo")
    {
        // 创建静态 TF 发布器
        broadcaster_ =
            std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

        // 创建 base_link -> laser 的坐标变换
        geometry_msgs::msg::TransformStamped transform;

        transform.header.stamp = this->get_clock()->now();

        // 父坐标系：小车底盘
        transform.header.frame_id = "base_link";

        // 子坐标系：雷达
        transform.child_frame_id = "laser";

        // 雷达相对于底盘的位置
        transform.transform.translation.x = 1.0;
        transform.transform.translation.y = 0.5;
        transform.transform.translation.z = 0.2;

        // 雷达绕 Z 轴旋转 90 度
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.70710678;
        transform.transform.rotation.w = 0.70710678;

        // 发布静态 TF
        broadcaster_->sendTransform(transform);

        RCLCPP_INFO(
            this->get_logger(),
            "Published static TF: base_link -> laser"
        );
    }

private:
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<LaserStaticBroadcaster>());

    rclcpp::shutdown();
    return 0;
}

