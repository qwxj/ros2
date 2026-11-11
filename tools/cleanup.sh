#!/usr/bin/env bash
# ============================================================
# 一键清理本项目的所有节点与残留进程
#
# 为什么需要它：
#   `ros2 launch` 按 Ctrl+C 时，子进程不一定全被杀掉 ——
#   TimerAction 延迟起的节点、LifecycleNode、ExecuteProcess 起的进程
#   都可能残留。残留会积累成多份实例：
#     · 多个 collision_monitor 抢 /cmd_vel_final → 车乱动或不动
#     · 多个 laser_scan_simulator     抢 /scan       → SLAM 收到矛盾数据
#     · 多个 map→odom 发布者                         → 车"跑出地图"
#   而 ROS 2 对这些情况**完全不报错**，只会看到
#   "nodes share an exact name" 的 WARNING。
#
# 用法：
#   ./tools/cleanup.sh          清理 + 显示剩余节点
#   ./tools/cleanup.sh -q       安静模式（只显示结果）
#
# 注意：本脚本会 kill 掉所有匹配到的进程，包括你手动起的节点。
# ============================================================

set +e   # 任何 pkill 找不到进程都会返回 1，不能让 set -e 中断

QUIET=0
[ "$1" = "-q" ] && QUIET=1

PATTERNS=(
  # launch 文件（先杀 launch，它会带走子进程）
  "robot.launch.py"
  "nav2_standalone.launch.py"
  "gazebo.launch.py"
  "bringup_all.launch.py"
  "slam_mini.launch.py"
  "amcl_mini.launch.py"
  "debug_map.launch.py"
  # ros2_control / 硬件接口 / 传感器
  "ros2_control_node"
  "laser_scan_simulator"
  "imu_sensor"
  "robot_state_publisher"
  # 控制链
  "cmd_vel"
  "collision_monitor"
  # 定位 / 建图
  "slam_mini_node"
  "amcl_mini_node"
  "ekf_local"
  "ekf_global"
  "ekf_node"
  "slam_toolbox"
  "map_server"
  "lifecycle_manager"
  # Nav2
  "nav2_"
  "controller_server"
  "planner_server"
  "bt_navigator"
  "behavior_server"
  "smoother_server"
  "waypoint_follower"
  "velocity_smoother"
  "map_saver"
  # 可视化 / 仿真
  "rviz2"
  "gzserver"
  "gzclient"
)

KILLED=0
for p in "${PATTERNS[@]}"; do
  if pgrep -f -- "$p" >/dev/null 2>&1; then
    n=$(pgrep -fc -- "$p" 2>/dev/null)
    [ "$QUIET" -eq 0 ] && printf '  杀 %-32s (%s 个进程)\n' "$p" "${n:-?}"
    pkill -9 -f -- "$p" >/dev/null 2>&1
    KILLED=$((KILLED + 1))
  fi
done

sleep 2

# ROS 2 daemon 会缓存节点列表，不清掉的话 `ros2 node list` 显示的是旧数据
timeout 5 ros2 daemon stop  >/dev/null 2>&1
timeout 5 ros2 daemon start >/dev/null 2>&1
sleep 2

echo ""
echo "════════ 清理结果 ════════"
if [ "$KILLED" -eq 0 ]; then
  echo "  没有发现需要清理的进程"
else
  echo "  清理了 $KILLED 组进程"
fi

REMAIN=$(ros2 node list 2>/dev/null | grep -v '^/transform_listener_impl_' | sort)
if [ -z "$REMAIN" ]; then
  echo "  剩余节点：无 ✅"
else
  echo "  ⚠️  仍有节点残留："
  echo "$REMAIN" | sed 's/^/    /'
  echo ""
  echo "  如果有重名的（同名出现多次），再执行一次本脚本。"
  echo "  如果还有，用下面的命令看是哪个进程："
  echo "    ps aux | grep -E 'ros2|robot' | grep -v grep"
fi

echo ""
echo "════════ 关键话题（都应该是 0 个发布者）════════"
for t in /scan /odom /map /cmd_vel /cmd_vel_final /tf; do
  out=$(timeout 3 ros2 topic info "$t" 2>/dev/null)
  if [ -z "$out" ]; then
    printf '  %-16s 话题不存在（等价于 0 个发布者）✅\n' "$t"
  else
    c=$(echo "$out" | grep -i 'Publisher count' | tr -dc '0-9')
    c=${c:-0}
    if [ "$c" = "0" ]; then
      printf '  %-16s 0 ✅\n' "$t"
    else
      printf '  %-16s %s ⚠️  还有发布者残留\n' "$t" "$c"
    fi
  fi
done
