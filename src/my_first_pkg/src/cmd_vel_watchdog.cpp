#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <chrono>

class CmdVelWatchdog : public rclcpp::Node
{
public:
    CmdVelWatchdog() : Node("cmd_vel_watchdog")
    {   
        timeout_sec_ = declare_parameter<double>("timeout_sec", 0.5);
        sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg){
                last_cmd_ = *msg;

                last_cmd_time_ = Clock::now();          // ← 新增：记下"什么时候收到的"
                has_cmd_ = true;                        // ← 新增：标记"当前有有效指令"

                RCLCPP_INFO(get_logger(), "收到指令:linear.x=%.2f angular.z=%.2f", msg->linear.x, msg->angular.z);
            }
        );

        pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_safe", 10);
        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            [this](){
                tick();
            }
        );
        RCLCPP_INFO(get_logger(), "cmd_vel_watchdog 已启动：/cmd_vel -> /cmd_vel_safe（超时 %.2f 秒）", timeout_sec_);
    }
private:
    using Clock = std::chrono::steady_clock; 
    void tick() {
        const double age_sec =
            std::chrono::duration<double>(Clock::now() - last_cmd_time_).count();

        // 超时了：只在"从有到无"这一刻打印一次，避免 20 Hz 刷屏
        if (has_cmd_ && age_sec > timeout_sec_) {
            has_cmd_ = false;
            RCLCPP_WARN(get_logger(), "%.2f 秒没收到指令，紧急停车", age_sec);
        }

        if (has_cmd_) {
            pub_->publish(last_cmd_);                   // 有指令：原样转发
        } else {
            pub_->publish(geometry_msgs::msg::Twist()); // 没指令：发全零
        }
    }

    double timeout_sec_ = 0.5;                      // ← 新增
    bool has_cmd_ = false;                          // ← 新增
    Clock::time_point last_cmd_time_ = Clock::now();// ← 新增

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;

    // 定时器
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Twist last_cmd_;
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelWatchdog>());
    rclcpp::shutdown();
    return 0;
}