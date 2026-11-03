#include <cmath>
#include <memory>
#include <string>
#include <chrono>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class PoseRepoter : public rclcpp::Node{

public:
    PoseRepoter() : Node("pose_repoter")
    {
        buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        auto time_tf = std::make_shared<tf2_ros::CreateTimerROS>(
            get_node_base_interface(), get_node_timers_interface()
        );
        buffer_->setCreateTimerInterface(time_tf);

        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(200), std::bind(&PoseRepoter::tick, this)
        );

    }

private:

    void tick()
    {
        const std::string target = "map";
        const std::string source = "base_link";

        if(!buffer_->canTransform(target, source, tf2::TimePointZero))
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "无法查询 %s → %s 的变换", target.c_str(), source.c_str());
            return ;
        }

        try{
            const auto tf = buffer_->lookupTransform(target, source, tf2::TimePointZero);
            const double yaw = tf2::getYaw(tf.transform.rotation);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "机器人在 %s 坐标系中的位置: x=%.2f, y=%.2f, yaw=%.2f", target.c_str(), tf.transform.translation.x, tf.transform.translation.y, yaw);
        }
        catch(const tf2::TransformException & ex)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "查询 %s → %s 的变换失败: %s", target.c_str(), source.c_str(), ex.what());
            return ;
        }
        
    }
    std::unique_ptr<tf2_ros::Buffer> buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseRepoter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}