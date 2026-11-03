#include <memory>
#include <chrono>
#include <mutex>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class CmdWatchDog : public rclcpp::Node
{
public:
    CmdWatchDog() : Node("cmd_watch_dog")
    {
        timeout_sec_ = declare_parameter<double>("timeout_sec", 0.5);
        sub_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        timer_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions sub_opt;
        sub_opt.callback_group = sub_group_;
        sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            [this](geometry_msgs::msg::Twist::SharedPtr m){
                on_cmd(m);
            }, sub_opt
        );

        pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_safe", 10);
        timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&CmdWatchDog::on_tick, this), timer_group_);
   }
        

private:
    using SteadyClock = std::chrono::steady_clock;

    void on_cmd(geometry_msgs::msg::Twist::SharedPtr p)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        last_cmd_ = *p;
        last_time_ = SteadyClock::now();
        has_cmd_ = true;
    }


    void on_tick()
    {
        std::lock_guard<std::mutex> lk(mtx_);
        const double age = std::chrono::duration<double>(
                                SteadyClock::now() - last_time_).count();
        if(has_cmd_ && age > timeout_sec_)
        {
            RCLCPP_WARN(get_logger(), "指令超时%.2fs,停车",age);
            has_cmd_ = false;
        }

        pub_->publish(has_cmd_ ? last_cmd_ : geometry_msgs::msg::Twist{});
        
    }


    double timeout_sec_{0.5};
    std::mutex mtx_;
    geometry_msgs::msg::Twist last_cmd_;
    SteadyClock::time_point last_time_{SteadyClock::now()};
    bool has_cmd_{false};

    rclcpp::CallbackGroup::SharedPtr sub_group_;
    rclcpp::CallbackGroup::SharedPtr timer_group_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

};


int main(int argc, char**argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(std::make_shared<CmdWatchDog>());
    exec.spin();
    rclcpp::shutdown();
    return 0;
}