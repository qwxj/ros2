#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <algorithm>
#include <sstream>


#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "rclcpp/rclcpp.hpp"

class ObStacleServer : public rclcpp::Node
{
public:
    ObStacleServer() : Node("obstacle_server")
    {

        sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::LaserScan::SharedPtr msg){
                on_scan(msg);
            }
        );

        srv_ = create_service<std_srvs::srv::Trigger>(
            "/get_obstacle", 
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res){
                on_request(req,res);
            }
        );
        
    }


private:
    void on_scan(sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        float best = std::numeric_limits<float>::infinity();

        int best_idx = -1;
        float best_angle = 0.0;

        for(size_t i=0;i<msg->ranges.size();i++)
        {
            const float r = msg->ranges[i];
            if(!std::isfinite(r)) continue;
            if(r < msg->range_min || r > msg->range_max) continue;
            if(r < best)
            {
                best = r;
                best_idx = static_cast<int>(i);
                best_angle = msg->angle_min + static_cast<float>(i) * msg->angle_increment;

            }
        }
        std::lock_guard<std::mutex> lk(mutex_);
        last_dist_ = best;
        last_angle_ = best_angle;
        has_data_ = (best_idx >= 0);
        
    }

    void on_request(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        (void)req;
        std::lock_guard<std::mutex> lk(mutex_);
        if(!has_data_)
        {
            res->success = false;
            res->message = "还未收到有效数据";
            return;
        }
        
        std::ostringstream oos;
        oos<<"dist="<<last_dist_<<",ngle_deg="<<(last_angle_*180/M_PI);
        res->success = true;
        res->message = oos.str();
    }

    std::mutex mutex_;
    float last_dist_{std::numeric_limits<float>::infinity()};
    float last_angle_{0.0f};
    bool has_data_{false};

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObStacleServer>());
    rclcpp::shutdown();
    return 0;
}