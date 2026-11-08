#include "mini_slam/ekf.hpp"

#include <cmath>

namespace mini_slam
{

double Ekf2D::wrapToPi(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a < -M_PI) {a += 2.0 * M_PI;}
  return a;
}

void Ekf2D::initialize(double x0, double y0, double yaw0)
{
  s_.setZero();
  s_(0) = x0;
  s_(1) = y0;
  s_(2) = wrapToPi(yaw0);

  // 初始不确定性：位置 0.5 m、朝向 0.22 rad
  // （真实场景里初始化时并不知道自己在哪，P 给太小会导致"死不认账"）
  P_.setZero();
  P_.diagonal() << 0.25, 0.25, 0.05, 0.1, 0.1;

  initialized_ = true;
}

void Ekf2D::predict(double dt, double sigma_v, double sigma_w)
{
  if (!initialized_) {return;}

  const double c = std::cos(s_(2));
  const double sy = std::sin(s_(2));
  const double v = s_(3);
  const double w = s_(4);

  // ---------- 非线性运动模型（匀速）----------
  s_(0) += v * c * dt;
  s_(1) += v * sy * dt;
  s_(2) = wrapToPi(s_(2) + w * dt);

  // ---------- 雅可比 F = df/dx ----------
  StateMat F = StateMat::Identity();
  F(0, 2) = -v * sy * dt;
  F(0, 3) = c * dt;
  F(1, 2) = v * c * dt;
  F(1, 3) = sy * dt;
  F(2, 4) = dt;

  // ---------- 过程噪声 Q ----------
  StateMat Q = StateMat::Zero();
  Q(0, 0) = sigma_v * dt;
  Q(1, 1) = sigma_v * dt;
  Q(2, 2) = sigma_w * dt;
  Q(3, 3) = sigma_v;
  Q(4, 4) = sigma_w;

  P_ = F * P_ * F.transpose() + Q;
}

void Ekf2D::updateGeneric(const Eigen::VectorXd & z,
                          const Eigen::MatrixXd & H,
                          const Eigen::MatrixXd & R)
{
  if (!initialized_) {return;}

  const Eigen::MatrixXd PHt = P_ * H.transpose();
  const Eigen::MatrixXd S = H * PHt + R;
  const Eigen::MatrixXd K = PHt * S.inverse();

  // ★ 必须是动态向量：z - H*s_ 的维度 = z 的行数（可能是 1/2/3），
  //   不是固定的 5。写成 StateVec 会触发 Eigen 的 resize 断言。
  const Eigen::VectorXd innovation = z - H * s_;

  s_ += K * innovation;
  s_(2) = wrapToPi(s_(2));
  P_ = (StateMat::Identity() - K * H) * P_;
}

void Ekf2D::updateOdomTwist(double vx, double vyaw, double sigma_v, double sigma_w)
{
  Eigen::VectorXd z(2);
  z << vx, vyaw;

  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, N);
  H(0, 3) = 1.0;
  H(1, 4) = 1.0;

  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(2, 2);
  R(0, 0) = sigma_v * sigma_v;
  R(1, 1) = sigma_w * sigma_w;

  updateGeneric(z, H, R);
}

void Ekf2D::updateYawRate(double yaw_rate, double sigma_w)
{
  Eigen::VectorXd z(1);
  z << yaw_rate;

  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, N);
  H(0, 4) = 1.0;

  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(1, 1) * sigma_w * sigma_w;

  updateGeneric(z, H, R);
}

void Ekf2D::updatePose(double x, double y, double yaw,
                       double sigma_xy, double sigma_yaw)
{
  Eigen::VectorXd z(3);
  // yaw 用最短路径残差，避免 ±π 附近跳变
  z << x, y, s_(2) + wrapToPi(yaw - s_(2));

  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, N);
  H(0, 0) = 1.0;
  H(1, 1) = 1.0;
  H(2, 2) = 1.0;

  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(3, 3);
  R(0, 0) = sigma_xy * sigma_xy;
  R(1, 1) = sigma_xy * sigma_xy;
  R(2, 2) = sigma_yaw * sigma_yaw;

  updateGeneric(z, H, R);
}

}  // namespace mini_slam