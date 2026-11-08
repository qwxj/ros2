#ifndef MINI_SLAM__POSE_GRAPH_HPP_
#define MINI_SLAM__POSE_GRAPH_HPP_

#include <vector>

#include "mini_slam/pose2d.hpp"

namespace mini_slam
{

/// 一条边 = 一个相对位姿观测
///   含义：在 from 的坐标系下，to 位于 (dx, dy, dyaw)
struct GraphEdge
{
  int from{-1};
  int to{-1};
  double dx{0.0};
  double dy{0.0};
  double dyaw{0.0};
  double w_trans{1.0};   // 平移权重（1/方差）
  double w_rot{1.0};     // 旋转权重
  bool is_loop{false};   // 是否是回环边
};

struct GraphNode
{
  Pose2D pose;
  bool fixed{false};     // 固定节点（通常是第 0 个）不参与优化
};

/// 位姿图 + Gauss-Seidel 松弛优化
///
/// 注意：真实 SLAM 用 g2o / ceres 做稀疏 Gauss-Newton。
/// 这里用"迭代松弛"——每轮把每条边的误差分给两端，重复多次。
/// 它不是精确最小二乘，但能收敛、能把回环拉闭合，且 100 行能讲清。
class PoseGraph
{
public:
  /// 加节点，返回 id（第 0 个默认 fixed）
  int addNode(double x, double y, double yaw);

  /// 加边（相对位姿观测）
  int addEdge(int from, int to, double dx, double dy, double dyaw,
              double w_trans = 1.0, double w_rot = 1.0, bool is_loop = false);

  /// 回环候选：找 node_id 附近（半径 radius 米）且间隔 >= min_gap 的历史节点
  std::vector<int> findLoopCandidates(int node_id, double radius, int min_gap) const;

  /// 迭代松弛优化，返回优化后的总误差
  double optimize(int iterations = 200, double alpha = 0.6);

  /// 当前总误差（加权平方和）
  double totalError() const;

  const Pose2D & pose(int id) const { return nodes_[id].pose; }
  void setPose(int id, double x, double y, double yaw);
  void setFixed(int id, bool f) { nodes_[id].fixed = f; }

  int size() const { return static_cast<int>(nodes_.size()); }
  int edgeCount() const { return static_cast<int>(edges_.size()); }
  const std::vector<GraphEdge> & edges() const { return edges_; }

private:
  void edgeError(const GraphEdge & e, double & ex, double & ey, double & eyaw) const;

  std::vector<GraphNode> nodes_;
  std::vector<GraphEdge> edges_;
};

}  // namespace mini_slam

#endif