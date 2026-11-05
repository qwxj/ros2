#include "qos_lab/qos_utils.hpp"

#include <sstream>

namespace qos_lab
{

rclcpp::QoS declare_qos(rclcpp::Node & node, const std::string & prefix)
{
  auto text = [&node, &prefix](const std::string & key, const std::string & fallback) {
      const std::string name = prefix + key;
      if (!node.has_parameter(name)) {
        return node.declare_parameter<std::string>(name, fallback);
      }
      return node.get_parameter(name).as_string();
    };

  auto integer = [&node, &prefix](const std::string & key, int fallback) {
      const std::string name = prefix + key;
      if (!node.has_parameter(name)) {
        return node.declare_parameter<int>(name, fallback);
      }
      return static_cast<int>(node.get_parameter(name).as_int());
    };

  const std::string reliability = text("reliability", "reliable");
  const std::string durability = text("durability", "volatile");
  const std::string history = text("history", "keep_last");
  const int depth = integer("depth", 10);
  const std::size_t bounded_depth = static_cast<std::size_t>(depth > 0 ? depth : 1);

  rclcpp::QoS qos(bounded_depth);

  if (reliability == "best_effort") {
    qos.best_effort();
  } else {
    qos.reliable();
  }

  if (durability == "transient_local") {
    qos.transient_local();
  } else {
    qos.durability_volatile();
  }

  if (history == "keep_all") {
    qos.keep_all();
  } else {
    qos.keep_last(bounded_depth);
  }

  return qos;
}

namespace
{
std::string duration_ms(const rmw_time_t & t)
{
  if (t.sec == 0 && t.nsec == 0) {
    return "none";
  }
  std::ostringstream oss;
  oss << (static_cast<double>(t.sec) * 1000.0 + static_cast<double>(t.nsec) / 1.0e6) << "ms";
  return oss.str();
}
}  // namespace

std::string describe(const rclcpp::QoS & qos)
{
  const rmw_qos_profile_t & p = qos.get_rmw_qos_profile();
  std::ostringstream oss;
  oss << (p.reliability == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT ? "best_effort" : "reliable")
      << "/" << (p.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL ?
    "transient_local" : "volatile")
      << "/" << (p.history == RMW_QOS_POLICY_HISTORY_KEEP_ALL ? "keep_all" : "keep_last")
      << " depth=" << p.depth
      << " deadline=" << duration_ms(p.deadline)
      << " lifespan=" << duration_ms(p.lifespan);
  return oss.str();
}

}  // namespace qos_lab
