
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <iomanip>

class QuaternionDemo : public rclcpp::Node
{
public:
    QuaternionDemo()
        : Node("quaternion_demo")
    {
        // 分别测试 0°、90°、180° 的 yaw 旋转
        print_quaternion(0.0);
        print_quaternion(M_PI / 2.0);
        print_quaternion(M_PI);

        // 再测试一个 45° 的 yaw 旋转
        print_quaternion(M_PI / 4.0);
    }

private:
    void print_quaternion(double yaw)
    {
        tf2::Quaternion q;

        // 参数依次为 roll、pitch、yaw，单位是弧度
        q.setRPY(0.0, 0.0, yaw);

        RCLCPP_INFO(
            this->get_logger(),
            "yaw = %.1f degrees | x=%.4f, y=%.4f, z=%.4f, w=%.4f",
            yaw * 180.0 / M_PI,
            q.x(), q.y(), q.z(), q.w());
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<QuaternionDemo>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
