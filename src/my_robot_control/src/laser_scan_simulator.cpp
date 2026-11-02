
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <functional>

struct Point2D
{
    double x;
    double y;
};

struct Segment
{
    Point2D A;
    Point2D B;
};

double cross(
    const Point2D &a,
    const Point2D &b)
{
    return a.x * b.y - a.y * b.x;
}

double raySegmentIntersection(
    const Point2D &origin,
    const Point2D &direction,
    const Point2D &A,
    const Point2D &B)
{
    Point2D E{
        B.x - A.x,
        B.y - A.y
    };

    Point2D AP{
        A.x - origin.x,
        A.y - origin.y
    };

    double denominator = cross(direction, E);

    if (std::abs(denominator) < 1e-9)
    {
        return std::numeric_limits<double>::infinity();
    }

    double t = cross(AP, E) / denominator;
    double u = cross(AP, direction) / denominator;

    if (t >= 0.0 && u >= 0.0 && u <= 1.0)
    {
        return t;
    }

    return std::numeric_limits<double>::infinity();
}

class LaserScanSimulator : public rclcpp::Node
{
public:
    LaserScanSimulator()
        : Node("laser_scan_simulator"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        publisher_ = this->create_publisher<
            sensor_msgs::msg::LaserScan>("/scan", 10);

        // 世界坐标系 odom 中的固定障碍物
        // 世界坐标系 odom 中的固定障碍物
        // 与 maps/map.pgm 中的障碍物坐标保持一致
        obstacles_ = {

            // ==============================
            // 房间四周墙壁的内侧边界
            // 地图范围：X[-5, 5], Y[-4, 4]
            // 墙壁厚度：0.15m
            // ==============================

            // 下墙内侧
            {{-4.85, -3.85}, {4.85, -3.85}},

            // 右墙内侧
            {{4.85, -3.85}, {4.85, 3.85}},

            // 上墙内侧
            {{4.85, 3.85}, {-4.85, 3.85}},

            // 左墙内侧
            {{-4.85, 3.85}, {-4.85, -3.85}},

            // ==============================
            // 障碍物 1
            // X[-2.5, -1.8], Y[-1.0, 1.5]
            // ==============================

            {{-2.5, -1.0}, {-1.8, -1.0}},
            {{-1.8, -1.0}, {-1.8, 1.5}},
            {{-1.8, 1.5}, {-2.5, 1.5}},
            {{-2.5, 1.5}, {-2.5, -1.0}},

            // ==============================
            // 障碍物 2
            // X[-0.2, 0.6], Y[-3.0, -1.0]
            // ==============================

            {{-0.2, -3.0}, {0.6, -3.0}},
            {{0.6, -3.0}, {0.6, -1.0}},
            {{0.6, -1.0}, {-0.2, -1.0}},
            {{-0.2, -1.0}, {-0.2, -3.0}},

            // ==============================
            // 障碍物 3
            // X[1.2, 2.8], Y[0.5, 1.2]
            // ==============================

            {{1.2, 0.5}, {2.8, 0.5}},
            {{2.8, 0.5}, {2.8, 1.2}},
            {{2.8, 1.2}, {1.2, 1.2}},
            {{1.2, 1.2}, {1.2, 0.5}},

            // ==============================
            // 障碍物 4
            // X[-0.5, 0.8], Y[2.0, 2.6]
            // ==============================

            {{-0.5, 2.0}, {0.8, 2.0}},
            {{0.8, 2.0}, {0.8, 2.6}},
            {{0.8, 2.6}, {-0.5, 2.6}},
            {{-0.5, 2.6}, {-0.5, 2.0}}
        };

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(
                &LaserScanSimulator::publish_scan,
                this));

        RCLCPP_INFO(
            get_logger(),
            "World-based laser simulator started");
    }

private:
    void publish_scan()
    {
        geometry_msgs::msg::TransformStamped transform;

        try
        {
            // 获取 odom -> laser_link 的变换
            transform = tf_buffer_.lookupTransform(
                "odom",
                "laser_link",
                tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(
                get_logger(),
                "TF lookup failed: %s",
                ex.what());

            return;
        }

        // 雷达在 odom 坐标系中的位置
        Point2D origin{
            transform.transform.translation.x,
            transform.transform.translation.y
        };

        // 雷达相对于 odom 的偏航角
        double yaw = tf2::getYaw(
            transform.transform.rotation);

        constexpr double PI = 3.14159265358979323846;

        sensor_msgs::msg::LaserScan scan;

        scan.header.stamp = now();
        scan.header.frame_id = "laser_link";

        scan.angle_min = -PI;
        scan.angle_max = PI;
        scan.angle_increment = PI / 180.0;

        scan.time_increment = 0.0;
        scan.scan_time = 0.1;

        scan.range_min = 0.12;
        scan.range_max = 8.0;

        const int ray_count = 360;

        scan.ranges.resize(ray_count);

        for (int i = 0; i < ray_count; ++i)
        {
            // 雷达坐标系中的角度
            double local_angle =
                scan.angle_min +
                i * scan.angle_increment;

            // 转换为 odom 坐标系中的角度
            double world_angle = local_angle + yaw;

            // 世界坐标系中的射线方向
            Point2D direction{
                std::cos(world_angle),
                std::sin(world_angle)
            };

            double min_distance =
                std::numeric_limits<double>::infinity();

            // 与所有固定障碍物求交
            for (const auto &obstacle : obstacles_)
            {
                double distance = raySegmentIntersection(
                    origin,
                    direction,
                    obstacle.A,
                    obstacle.B);

                if (distance < min_distance)
                {
                    min_distance = distance;
                }
            }

            if (!std::isfinite(min_distance) ||
                min_distance > scan.range_max ||
                min_distance < scan.range_min)
            {
                scan.ranges[i] =
                    std::numeric_limits<float>::infinity();
            }
            else
            {
                scan.ranges[i] = min_distance;
            }
        }

        publisher_->publish(scan);

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Laser position: x=%.2f, y=%.2f, yaw=%.2f",
            origin.x,
            origin.y,
            yaw);
    }

    rclcpp::Publisher<
        sensor_msgs::msg::LaserScan>::SharedPtr publisher_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<Segment> obstacles_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<LaserScanSimulator>());

    rclcpp::shutdown();

    return 0;
}
