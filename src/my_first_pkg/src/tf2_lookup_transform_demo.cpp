#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class LookupTransformDemo : public rclcpp::Node
{
public:
    LookupTransformDemo()
        : Node("lookup_transform_demo")
    {
        // 创建 TF Buffer
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(
            this->get_clock());

        // 创建 TF Listener
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(
                *tf_buffer_);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(
                &LookupTransformDemo::lookup_transform,
                this));
    }

private:
    void lookup_transform()
    {
        try
        {
            auto transform = tf_buffer_->lookupTransform(
                "base_link",
                "laser",
                tf2::TimePointZero);

            RCLCPP_INFO(
                this->get_logger(),
                "Translation: x=%.2f, y=%.2f, z=%.2f",
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z);

            RCLCPP_INFO(
                this->get_logger(),
                "Rotation: x=%.4f, y=%.4f, z=%.4f, w=%.4f",
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w);
        }
        catch (const tf2::TransformException & ex)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "TF lookup failed: %s", ex.what());
        }
    }

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<LookupTransformDemo>());

    rclcpp::shutdown();
    return 0;
}