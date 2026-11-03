#include <cmath>
#include <memory>
#include <algorithm>
#include <functional>


#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class GoToPoint : public rclcpp::Node
{
public:
    GoToPoint() : Node("Go_to_point")
    {
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10 ,
            [this](nav_msgs::msg::Odometry::SharedPtr odom){
                do_odom(odom);
            }
        );

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10
        );

        done_pub_ = create_publisher<std_msgs::msg::Bool>(
            "/goal_done", 10
        );

        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&GoToPoint::tick, this)
        );

    }

private:    
    enum class State{WAIT_ODOM, DRIVE, TURN, DONE};

    void do_odom(nav_msgs::msg::Odometry::SharedPtr odom)
    {
        x_ = odom->pose.pose.position.x;
        y_ = odom->pose.pose.position.y;
        yaw_ = tf2::getYaw(odom->pose.pose.orientation);
        if(state_ == State::WAIT_ODOM)
        {
            state_ = State::DRIVE;
        }
    }

    void tick()
    {
        geometry_msgs::msg::Twist cmd;
        switch(state_)
        {
            case State::WAIT_ODOM:
                break;
            case State::DRIVE:
            {
                const double dx = target_x_ - x_;
                const double dy = target_y_ - y_;
                const double dist = std::hypot(dx, dy);

                if(dist < kArriveTol){
                    turn_target_ = yaw_ + M_PI / 2;
                    state_ = State::TURN;
                    break;
                }

                const double heading = std::atan2(dy, dx);
                cmd.linear.x = kLinearSpeed;
                cmd.angular.z = std::clamp(kHeadingGain * (heading - yaw_), -0.5, 0.5);
                break;
            }

            case State::TURN:{
                double err = turn_target_ - yaw_;
                while(err > M_PI) err -= 2*M_PI;
                while(err < -M_PI) err+=2*M_PI;
                if(std::fabs(err) < kYawTol){
                    state_ = State::DONE;
                    std_msgs::msg::Bool d;
                    d.data = true;
                    done_pub_->publish(d);
                }
                cmd.angular.z = std::clamp(kYawGain * err, -0.6, 0.6);
                break;
            }

            case State::DONE : {
                break;
            }
        }

        cmd_pub_->publish(cmd);
    }


    static constexpr double kLinearSpeed = 0.15;
    static constexpr double kHeadingGain = 1.5;
    static constexpr double kYawGain = 1.5;
    static constexpr double kArriveTol = 0.10;
    static constexpr double kYawTol = 0.05;
    static constexpr double target_x_ = 2.0;
    static constexpr double target_y_ = 0.0;

    State state_{State::WAIT_ODOM};
    double x_{0}, y_{0}, yaw_{0};
    double turn_target_{0};


    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr done_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoToPoint>());
    rclcpp::shutdown();
    return 0;
}