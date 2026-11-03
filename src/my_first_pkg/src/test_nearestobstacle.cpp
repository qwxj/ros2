#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cmath>
#include <limits>
#include <memory>


class NearestObstacle : public rclcpp::Node
{
public:
    NearestObstacle() : Node("nearest_obstacle")
    {
        subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::LaserScan::SharedPtr msg){
                on_scan(msg);
            }
        );

        publisher_ = create_publisher<std_msgs::msg::Float32>(
            "/nearest_obstacle", 10
        );
    }

private:
    void on_scan(sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        float best = std::numeric_limits<float>::infinity();
        
        for(float r: msg->ranges)
        {
            if(!std::isfinite(r)) continue;
            if(r < msg->range_min || r > msg->range_max) continue;
            if(best > r) best = r;
        }

        std_msgs::msg::Float32 out;
        out.data = best;
        publisher_->publish(out);

    }    

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;

};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NearestObstacle>());
    rclcpp::shutdown();
    return 0;
}