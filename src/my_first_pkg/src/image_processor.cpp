#include <memory>
#include <vector>

#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class ImageProcessor : public rclcpp::Node
{
public:
    ImageProcessor() : Node("image_processor")
    {   
        // ① 声明参数（带默认值）
        canny_low_  = declare_parameter<int>("canny_low", 50);
        canny_high_ = declare_parameter<int>("canny_high", 150);

        // ② 注册"参数被修改时"的回调 —— 没有它，param set 改了值你代码里也不知道
        param_cb_ = add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> & params) {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;

                // 先在局部变量上试算，校验通过才真正生效
                int lo = canny_low_;
                int hi = canny_high_;
                for (const auto & p : params) {
                    if (p.get_name() == "canny_low") {
                        lo = static_cast<int>(p.as_int());
                    } else if (p.get_name() == "canny_high") {
                        hi = static_cast<int>(p.as_int());
                    }
                }

                if (lo < 0 || hi > 255 || lo >= hi) {
                    result.successful = false;
                    result.reason = "要求 0 <= canny_low < canny_high <= 255";
                    RCLCPP_WARN(get_logger(), "拒绝参数修改: %s", result.reason.c_str());
                    return result;
                }

                canny_low_  = lo;
                canny_high_ = hi;
                RCLCPP_INFO(get_logger(), "参数已更新: canny_low=%d, canny_high=%d",
                            canny_low_, canny_high_);
                return result;
            });

        pub_ = create_publisher<sensor_msgs::msg::Image>("/image_processed", 10);
        sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            [this](const sensor_msgs::msg::Image::SharedPtr msg){
                // ROS 图像 -> cv::Mat，顺便把 rgb8 转成 bgr8
                cv_bridge::CvImagePtr cv_ptr;
                try {
                    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
                } catch (const cv_bridge::Exception & e) {
                    RCLCPP_ERROR(get_logger(), "cv_bridge 转换失败: %s", e.what());
                    return;
                }

                const cv::Mat & img = cv_ptr->image;


                // ② OpenCV 处理：BGR -> 灰度 -> Canny 边缘
                cv::Mat gray, edges;
                cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
                cv::Canny(gray, edges, canny_low_, canny_high_);

                // ③ 把 cv::Mat 打包回 ROS 消息
                cv_bridge::CvImage out_msg;
                out_msg.header   = msg->header;    // 继承时间戳和 frame_id
                out_msg.encoding = "mono8";        // 单通道 8 位
                out_msg.image    = edges;
                pub_->publish(*out_msg.toImageMsg());


                // 每秒最多打一条，否则 10 Hz 会刷爆终端
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                    "收到图像: %d x %d, 通道=%d, 编码=%s, 深度=%d, 连续=%d",
                    img.cols, img.rows, img.channels(),
                    cv_ptr->encoding.c_str(), img.depth(), img.isContinuous());
            });

            RCLCPP_INFO(get_logger(),
                "image_processor 已启动，Canny(%d, %d)，等待 /camera/image_raw",
                canny_low_, canny_high_);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_; // 发布处理的图像

    int canny_low_  = 50;
    int canny_high_ = 150;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageProcessor>());
    rclcpp::shutdown();
    return 0;
}