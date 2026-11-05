// L1 公共工具：把命令行参数翻译成 QoS，并打印出来。
// 设计要点：所有参数都带前缀（pub_ / sub_），这样同一套代码能分别配置两端。
#ifndef QOS_LAB__QOS_UTILS_HPP_
#define QOS_LAB__QOS_UTILS_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"

namespace qos_lab
{
/// 读取 <prefix>reliability / <prefix>durability / <prefix>history / <prefix>depth
rclcpp::QoS declare_qos(rclcpp::Node & node, const std::string & prefix = "");

/// 打印成一行
std::string describe(const rclcpp::QoS & qos);
}  // namespace qos_lab

#endif  // QOS_LAB__QOS_UTILS_HPP_
