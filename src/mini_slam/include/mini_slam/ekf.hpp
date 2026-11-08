#ifndef MINI_SLAM__EKF_HPP_
#define MINI_SLAM__EKF_HPP_

#include <Eigen/Dense>

namespace mini_slam
{

/// 二维 EKF：状态 = [x, y, yaw, vx, vyaw]
///
/// 对应 robot_localization 的 two_d_mode=true：
///   15 维状态只保留 5 维（x, y, yaw, vx, vyaw）
///   z / roll / pitch 全部锁死为 0
class Ekf2D
{
public:
  static constexpr int N = 5;
  using StateVec = Eigen::Matrix<double, N, 1>;
  using StateMat = Eigen::Matrix<double, N, N>;

  void initialize(double x0, double y0, double yaw0);

  /// 预测：匀速模型 x += v*cos(yaw)*dt ...
  void predict(double dt, double sigma_v, double sigma_w);

  /// 更新：里程计的速度观测 [vx, vyaw]
  void updateOdomTwist(double vx, double vyaw, double sigma_v, double sigma_w);

  /// 更新：IMU 的角速度观测 [vyaw]
  void updateYawRate(double yaw_rate, double sigma_w);

  /// 更新：绝对位姿观测 [x, y, yaw]（AMCL / GPS）
  void updatePose(double x, double y, double yaw,
                  double sigma_xy, double sigma_yaw);

  double x()   const { return s_(0); }
  double y()   const { return s_(1); }
  double yaw() const { return s_(2); }
  double vx()  const { return s_(3); }
  double vyaw() const { return s_(4); }

  const StateVec & state() const { return s_; }
  const StateMat & covariance() const { return P_; }
  bool initialized() const { return initialized_; }

  static double wrapToPi(double a);

private:
  void updateGeneric(const Eigen::VectorXd & z,
                     const Eigen::MatrixXd & H,
                     const Eigen::MatrixXd & R);

  StateVec s_ = StateVec::Zero();
  StateMat P_ = StateMat::Identity();
  bool initialized_ = false;
};

}  // namespace mini_slam

#endif