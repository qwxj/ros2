#ifndef MY_ROBOT_CONTROL__GROUND_TRUTH_HPP_
#define MY_ROBOT_CONTROL__GROUND_TRUTH_HPP_

// 「真实世界」的进程级共享状态。
//
// 为什么可以这样：ros2_control 会把所有 hardware component（DiffDriveHardware
// 和 ImuSensor）加载进【同一个进程】（ros2_control_node）的【同一个 .so】里。
// 所以可以用一个进程内单例来模拟「轮子在转 -> 世界在动 -> IMU 感受到了」。
// 真实硬件里这个角色由物理世界承担。
namespace my_robot_control
{

struct GroundTruth
{
  // 轮子角速度（rad/s），由 DiffDriveHardware::read() 写入
  double left_vel = 0.0;
  double right_vel = 0.0;

  // 真实偏航角（rad），由 DiffDriveHardware::read() 积分
  double yaw = 0.0;

  // 真实偏航角速度（rad/s），由轮速差算出
  double yaw_rate = 0.0;
};

// 进程内单例
inline GroundTruth & ground_truth()
{
  static GroundTruth instance;
  return instance;
}

}  // namespace my_robot_control

#endif  // MY_ROBOT_CONTROL__GROUND_TRUTH_HPP_
