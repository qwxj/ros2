#include "mini_slam/pose_graph.hpp"

#include <cmath>

namespace mini_slam
{

int PoseGraph::addNode(double x, double y, double yaw)
{
  const int id = static_cast<int>(nodes_.size());
  GraphNode n;
  n.pose.x = x;
  n.pose.y = y;
  n.pose.yaw = wrapToPi(yaw);
  n.fixed = (id == 0);      // 第一个节点固定，防止整体漂移
  nodes_.push_back(n);
  return id;
}

int PoseGraph::addEdge(int from, int to, double dx, double dy, double dyaw,
                       double w_trans, double w_rot, bool is_loop)
{
  GraphEdge e;
  e.from = from;
  e.to = to;
  e.dx = dx;
  e.dy = dy;
  e.dyaw = wrapToPi(dyaw);
  e.w_trans = w_trans;
  e.w_rot = w_rot;
  e.is_loop = is_loop;
  edges_.push_back(e);
  return static_cast<int>(edges_.size()) - 1;
}

void PoseGraph::setPose(int id, double x, double y, double yaw)
{
  nodes_[id].pose.x = x;
  nodes_[id].pose.y = y;
  nodes_[id].pose.yaw = wrapToPi(yaw);
}

std::vector<int> PoseGraph::findLoopCandidates(int node_id, double radius, int min_gap) const
{
  std::vector<int> out;
  if (node_id < 0 || node_id >= size()) {return out;}

  const Pose2D & cur = nodes_[node_id].pose;
  for (int i = 0; i < node_id - min_gap; ++i) {
    const Pose2D & p = nodes_[i].pose;
    if (std::hypot(p.x - cur.x, p.y - cur.y) <= radius) {out.push_back(i);}
  }
  return out;
}

void PoseGraph::edgeError(const GraphEdge & e,
                          double & ex, double & ey, double & eyaw) const
{
  const Pose2D & a = nodes_[e.from].pose;
  const Pose2D & b = nodes_[e.to].pose;

  // 由 a 的当前位姿推出 b "应该" 在哪
  const double c = std::cos(a.yaw);
  const double s = std::sin(a.yaw);
  const double want_x = a.x + c * e.dx - s * e.dy;
  const double want_y = a.y + s * e.dx + c * e.dy;
  const double want_yaw = a.yaw + e.dyaw;

  ex = want_x - b.x;
  ey = want_y - b.y;
  eyaw = wrapToPi(want_yaw - b.yaw);
}

double PoseGraph::totalError() const
{
  double total = 0.0;
  for (const auto & e : edges_) {
    double ex = 0.0, ey = 0.0, eyaw = 0.0;
    edgeError(e, ex, ey, eyaw);
    total += e.w_trans * (ex * ex + ey * ey) + e.w_rot * eyaw * eyaw;
  }
  return total;
}

double PoseGraph::optimize(int iterations, double alpha)
{
  if (nodes_.empty() || edges_.empty()) {return 0.0;}

  for (int it = 0; it < iterations; ++it) {
    for (const auto & e : edges_) {
      double ex = 0.0, ey = 0.0, eyaw = 0.0;
      edgeError(e, ex, ey, eyaw);

      const bool can_a = !nodes_[e.from].fixed;
      const bool can_b = !nodes_[e.to].fixed;

      // 误差按权重分给两端
      double wa = can_a ? 0.5 : 0.0;
      double wb = can_b ? (can_a ? 0.5 : 1.0) : 0.0;
      const double norm = wa + wb;
      if (norm <= 0.0) {continue;}
      wa /= norm;
      wb /= norm;

      if (can_a) {
        nodes_[e.from].pose.x -= alpha * wa * ex;
        nodes_[e.from].pose.y -= alpha * wa * ey;
        nodes_[e.from].pose.yaw = wrapToPi(nodes_[e.from].pose.yaw - alpha * wa * eyaw);
      }
      if (can_b) {
        nodes_[e.to].pose.x += alpha * wb * ex;
        nodes_[e.to].pose.y += alpha * wb * ey;
        nodes_[e.to].pose.yaw = wrapToPi(nodes_[e.to].pose.yaw + alpha * wb * eyaw);
      }
    }
  }
  return totalError();
}

}  // namespace mini_slam