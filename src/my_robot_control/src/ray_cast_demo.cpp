
#include <iostream>
#include <cmath>
#include <limits>

struct Point2D
{
    double x;
    double y;
};

double cross(
    const Point2D &a,
    const Point2D &b)
{
    return a.x * b.y - a.y * b.x;
}

// 返回射线与线段的最近交点距离
// 没有交点时返回 infinity
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


#include <iostream>
#include <vector>
#include <cmath>
#include <limits>

// 一条障碍物线段
struct Segment
{
    Point2D A;
    Point2D B;
};


int main()
{
    Point2D origin{0.0, 0.0};

    // 世界中的多个障碍物
    std::vector<Segment> obstacles = {
        {{2.0, -2.0}, {2.0, 2.0}},
        {{4.0, -1.0}, {4.0, 1.0}},
        {{-2.0, 1.0}, {-2.0, 3.0}},
        {{-3.0, -2.0}, {-1.0, -2.0}},
        {{1.0, 3.0}, {3.0, 3.0}}
    };

    const double PI = 3.14159265358979323846;

    const double angle_min = -PI;
    const double angle_increment = PI / 180.0;

    const int ray_count = 360;

    const double range_max = 8.0;

    std::vector<double> ranges(ray_count);

    for (int i = 0; i < ray_count; ++i)
    {
        double angle =
            angle_min + i * angle_increment;

        // 当前射线方向
        Point2D direction{
            std::cos(angle),
            std::sin(angle)
        };

        double min_distance =
            std::numeric_limits<double>::infinity();

        // 检查所有障碍物
        for (const auto &obstacle : obstacles)
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

        // 超出量程或没有障碍物
        if (min_distance > range_max)
        {
            ranges[i] = std::numeric_limits<double>::infinity();
        }
        else
        {
            ranges[i] = min_distance;
        }
    }

    // 输出部分扫描结果
    for (int i = 0; i < ray_count; i += 30)
    {
        double angle =
            angle_min + i * angle_increment;

        std::cout
            << "角度: "
            << angle * 180.0 / PI
            << "°，距离: ";

        if (std::isfinite(ranges[i]))
        {
            std::cout << ranges[i] << " m\n";
        }
        else
        {
            std::cout << "无障碍物\n";
        }
    }

    std::cout << "扫描完成，射线数量: "
              << ranges.size() << std::endl;

    return 0;
}
