# L0 工程层：一个包从源码到 `ros2 run` 的全链路

> **目标**：不再"能跑就行"，而是每一步都知道为什么这样、坏了第一眼看哪里。
> **时长**：5 天 × 2 h（含 4 个实验）。
> **产出**：① 能脱稿画并讲清五段链路；② `robot_math` + `robot_math_demo` 跑通；③ 四个破坏性实验的报错截图；④ 自检 10 题答对 ≥ 8。

---

## 1. 五段链路（先记住这张图）

```
src/<pkg>/package.xml + CMakeLists.txt
   │  ① 发现 & 排序    colcon 读 package.xml 建依赖图，拓扑排序
   ▼
build/<pkg>/            ② 配置 & 编译    CMake configure → build
   │
   ▼
install/<pkg>/          ③ 安装（--symlink-install 时这里是符号链接）
   │
   ▼
source install/setup.bash   ④ 把 prefix 塞进 AMENT_PREFIX_PATH 等环境变量
   │
   ▼
ros2 run <pkg> <exe>    ⑤ ament index 查 prefix → 找 lib/<pkg>/<exe> → 执行
```

一句话：**colcon 只负责调度构建，真正的知识在"install 里有什么"和"环境变量指向哪"。**

绝大多数 "找不到包 / 找不到可执行 / 链接不上" 的问题，都出在第 ③ 步和第 ④ 步，而不是代码。

---

## 2. 构建：colcon 不是构建系统

它是**多包调度器**：发现包 → 按依赖拓扑排序 → 逐包调用 CMake/ament。

| 目录 | 内容 | 何时看它 |
|---|---|---|
| `build/<pkg>/` | CMake 中间产物、Makefile、编译缓存 | 增量编译异常、改了 CMake 不生效 |
| `install/<pkg>/` | 最终前缀，运行时的一切都从这里来 | 90% 的"找不到"问题 |
| `log/latest_build/<pkg>/` | 上一次构建的完整命令与输出 | 复现别人的构建 |

**常用参数**

| 参数 | 作用 |
|---|---|
| `--packages-select a b` | 只构建指定包 |
| `--packages-up-to a` | 构建 a 及其全部依赖 |
| `--symlink-install` | install 里放软链；Python/launch/yaml 改了不用重编 |
| `--merge-install` | 所有包装进同一个前缀（老 catkin 风格），环境变量短但隔离差 |
| `--cmake-args -DCMAKE_BUILD_TYPE=Debug` | 透传给 CMake |
| `--event-handlers console_direct+` | 实时看每个包的完整输出 |

**隔离构建的代价**：每个包有独立前缀，所以分两次构建时，**必须先 source 前一个包的 install**：

```bash
colcon build --packages-select robot_math
source install/setup.bash          # ← 少了这行，下一步 find_package 必失败
colcon build --packages-select robot_math_demo
```

一次构建多个包时 colcon 会自动把依赖包的前缀注入，所以 `--packages-up-to robot_math_demo` 不需要中间 source。

---

## 3. install/ 的目录约定

`robot_math` 装完之后长这样：

```
install/robot_math/
├── include/robot_math/diff_drive.hpp          ← 头文件
├── lib/librobot_math.so                       ← 库
├── share/robot_math/
│   ├── package.xml                            ← 复制一份，供 ros2 pkg xml 读
│   ├── cmake/robot_mathConfig.cmake           ← 消费端 find_package 找的就是它
│   ├── cmake/robot_mathConfig-version.cmake
│   └── cmake/export_robot_mathExport.cmake    ← imported target 的定义（文件名以实际为准）
└── share/ament_index/resource_index/packages/robot_math   ← 空文件，"此前缀有此包"的标记
```

`ros2 run` 的实现就三行：`ament_index` 查包 → 得到 prefix → 执行 `<prefix>/lib/<pkg>/<exe>`。

**所以 C++ 可执行必须装到 `lib/${PROJECT_NAME}`**，不是 `bin/`。这是 ROS 2 与普通 CMake 工程最大的目录约定差异。

---

## 4. 导出：三条 CMake 宏到底各管什么

| 宏 | 作用 | 不写的后果 |
|---|---|---|
| `ament_export_targets(export_x HAS_LIBRARY_TARGET)` | 把 `install(TARGETS ... EXPORT export_x)` 里的 target 登记进 `<pkg>Config.cmake`，消费端得到 `robot_math::robot_math` | 消费端报 "target was not found" |
| `ament_export_include_directories(include)` | 老式：只导出 `<pkg>_INCLUDE_DIRS` 变量 | 用老式写法的消费端找不到头文件 |
| `ament_export_dependencies(rclcpp ...)` | 让消费端 `find_package` 时自动 `find_package` 你的依赖 | 头文件缺 / 链接 undefined reference |

**消费端两种写法，必须知道区别：**

```cmake
# 老式：读目标包导出的变量
ament_target_dependencies(my_node rclcpp)

# 新式：imported target，自带 include/link/编译选项
target_link_libraries(my_node rclcpp::rclcpp robot_math::robot_math)
```

新式更可靠（interface 传递完整），也是现在官方推荐。老式还在大量存量代码里，面试会问"有什么区别"。

**验收动作**：`cat install/robot_math/share/robot_math/cmake/robot_mathConfig.cmake`，找到 `include(...)` 那一行，再 `cat` 它指向的文件，看到 `add_library(robot_math::robot_math SHARED IMPORTED)` 就算懂了。

---

## 5. 环境：`source` 到底改了什么

| 变量 | 谁在用 | 顺序含义 |
|---|---|---|
| `AMENT_PREFIX_PATH` | ament index、`ros2 pkg` | **靠前的优先**，overlay 覆盖 underlay |
| `COLCON_PREFIX_PATH` | colcon 自身 | 本次构建的工作空间 |
| `LD_LIBRARY_PATH` | 动态链接器 | 找不到 `.so` 时看它 |
| `PYTHONPATH` | Python 包 import | Python 包改了不生效时看它 |
| `PATH` | `ros2` 命令、部分脚本 | — |

**`setup.bash` vs `local_setup.bash`**

- `setup.bash`：会先按记录的顺序 source 它依赖的所有 underlay（Humble 本体等），再设自己。
- `local_setup.bash`：只设当前 prefix，不碰 underlay。

多个工作空间叠加时用 `local_setup.bash` 更可控；日常用 `setup.bash` 更方便。

**验证 overlay 顺序**：

```bash
echo $AMENT_PREFIX_PATH | tr ':' '\n' | head -5
ros2 pkg prefix robot_math      # 打印的路径 = 实际生效的那个
```

---

## 6. package.xml 的依赖标签

| 标签 | 含义 | 典型场景 |
|---|---|---|
| `buildtool_depend` | 构建工具 | `ament_cmake` / `ament_python`（几乎必有） |
| `build_depend` | 只在编译期需要 | 头文件库、代码生成器 |
| `build_export_depend` | 编译我需要，且**我的头文件也 include 它** | 你的公共头文件里 `#include <rclcpp/rclcpp.hpp>` |
| `exec_depend` | 只在运行期需要 | launch 文件、插件、被 `ros2 run` 调起的节点 |
| `depend` | 上面三个的合集 | 90% 的情况用它就够了 |
| `test_depend` | 测试期 | `ament_lint_auto`、`ament_cmake_gtest` |
| `doc_depend` | 文档 | 少见 |

**最容易踩的两个**：

1. 头文件里 include 了某库，却只写了 `exec_depend` → 别人编译你时找不到头文件。
2. launch 里 `Node(package='foo')`，但没写 `exec_depend foo` → rosdep 不装，运行时才炸。

`<export><build_type>` 决定 colcon 用哪套构建系统（`ament_cmake` / `ament_python`），写错会直接"包存在但构建不出来"。

---

## 7. ament_cmake vs ament_python

| | `ament_cmake` | `ament_python` |
|---|---|---|
| 适用 | C++、混合 | 纯 Python |
| 入口 | `CMakeLists.txt` + `ament_package()` | `setup.py` + `setup.cfg` |
| 可执行 | `install(TARGETS ... DESTINATION lib/${PROJECT_NAME})` | `entry_points={'console_scripts': [...]}` |
| 资源文件 | `install(DIRECTORY launch config ...)` | `data_files=[('share/<pkg>/launch', [...])]` |
| 改了要重编？ | 要 | `--symlink-install` 下不用 |

对照你仓库里的 `src/my_robot_tasks/`：

- `setup.cfg` 里的 `install_scripts=lib/my_robot_tasks` —— 这就是 Python 脚本也被装到 `lib/<pkg>/` 的原因，`ros2 run` 才找得到。
- `resource/my_robot_tasks`（空文件）—— ament index 的"此包存在"标记，C++ 包由 `ament_package()` 自动生成，Python 包要手写。

---

## 8. 实验 A：读懂现有包（15 min）

打开 `src/my_robot_control/CMakeLists.txt`，逐条回答：

1. `find_package(...)` 里哪些是**真的**被代码 include 的？（对照 `src/*.cpp` 的 include）
2. `ament_target_dependencies` 与 `target_include_directories` 各解决什么？
3. 第 84 行 `ament_export_targets` 和第 85 行 `ament_export_include_directories` —— **重复了吗？** 为什么？
4. 第 42 行 `pluginlib_export_plugin_description_file` 生成/安装了什么东西？（去 `install/my_robot_control/share/my_robot_control/` 里找那个 xml）
5. 为什么 `laser_scan_simulator` 装到 `lib/${PROJECT_NAME}`，而库装到 `lib`？

> 第 3 题答案：`HAS_LIBRARY_TARGET` 已经让 imported target 带上 include 路径，`ament_export_include_directories` 是给老式消费端兼容用的，属于冗余但无害。知道"为什么可以删"比知道"为什么要写"更重要。

---

## 9. 实验 B：建库 + 消费（30 min）

仓库里已备好两个包：`src/robot_math`（纯 C++ 库）和 `src/robot_math_demo`（消费它的 rclcpp 节点）。

```bash
cd ~/ros2
source /opt/ros/humble/setup.bash
rm -rf build/robot_math build/robot_math_demo install/robot_math install/robot_math_demo

colcon build --symlink-install --packages-up-to robot_math_demo \
  --event-handlers console_direct+

source install/setup.bash
ros2 run robot_math_demo wheel_speed_node
```

另开终端：

```bash
source ~/ros2/install/setup.bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3}, angular: {z: 0.5}}"
ros2 topic echo /wheel_speed
```

**预期**（r=0.0325, b=0.205）：`L = (0.3 - 0.5×0.1025)/0.0325 ≈ 7.65`，`R ≈ 10.81` rad/s。

跑通后立刻做三件事：

```bash
ros2 pkg prefix robot_math
ros2 pkg executables robot_math_demo
cat install/robot_math/share/robot_math/cmake/robot_mathConfig.cmake
```

---

## 10. 实验 C：破坏性实验（1 h，本层最重要）

**学习效率最高的动作，是把对的改错，看它怎么报。**

每个子实验：改一处 → 重新 build → 记录**完整报错** → 解释原因 → 改回来。

| # | 改什么 | 预期症状 | 说明 |
|---|---|---|---|
| C-1 | 注释掉 `ament_export_targets(...)` | demo 配置阶段报 `target "robot_math::robot_math" ... not found` | Config.cmake 还在，但里面没有 target 定义 |
| C-2 | 删掉 `install(DIRECTORY include/ ...)` | demo **编译**报 `robot_math/diff_drive.hpp: No such file or directory` | 库自己能编（用 BUILD_INTERFACE），消费端找不到 install 里的头文件 |
| C-3 | 删掉 `install(TARGETS ... EXPORT ...)` | 链接报 `undefined reference to robot_math::DiffDriveKinematics::...`，或 CMake 报 imported target 指向不存在的文件 | 库没被安装 |
| C-4 | demo 的 `DESTINATION lib/${PROJECT_NAME}` 改成 `bin` | `ros2 run robot_math_demo wheel_speed_node` → **No executable found** | 最常见的一个坑 |
| C-5 | 把 `--symlink-install` 去掉，改 `demo_node.cpp` 里一行日志再 build | 改了不生效 / 要重编 | 软链 vs 拷贝的区别 |

把 C-1 到 C-4 的报错原文抄进笔记，以后现场看到就能秒判。

---

## 11. 实验 D：环境与 overlay 追踪（30 min）

```bash
# 1. 当前环境里有哪些前缀，顺序如何
echo $AMENT_PREFIX_PATH | tr ':' '\n'

# 2. 某个包实际来自哪个前缀
ros2 pkg prefix robot_math
ros2 pkg prefix rclcpp

# 3. 包的可执行、xml 元信息
ros2 pkg executables robot_math_demo
ros2 pkg xml robot_math | head -15

# 4. 库到底被链接到哪个 .so
ldd install/robot_math_demo/lib/robot_math_demo/wheel_speed_node | grep robot_math
```

**overlay 实验**：新建 `/tmp/ws2/src/robot_math`（复制一份，改 `package.xml` 的版本号），单独 build，然后按不同顺序 source 两个工作空间，用 `ros2 pkg prefix robot_math` 观察谁生效。

结论应该是：**后 source 的在前面，所以后 source 的赢**。

---

## 12. 常见坑

1. 改了 Python/launch 没生效 → 没加 `--symlink-install`。
2. `ros2 run` 报 No executable found → 可执行没装到 `lib/<pkg>/`。
3. `find_package` 成功但链接 undefined reference → `ament_export_dependencies` 漏了。
4. 头文件找不到 → `install(DIRECTORY include/ ...)` 漏了，或 `INSTALL_INTERFACE` 路径与安装路径不一致。
5. 忘记 `source install/setup.bash` → 用的还是旧包（改完代码行为没变，先怀疑这个）。
6. `--merge-install` 与隔离 install 混用后旧文件残留 → `rm -rf build install` 重来。
7. 包名与目录名不一致、包名带大写 → colcon 警告甚至找不到。
8. 想临时跳过某个目录 → 在里面放一个空的 `COLCON_IGNORE`。

---

## 13. 自检 10 题

先自己答，再对照文末答案。

1. `install/` 里为什么同时有 `setup.bash` 和 `local_setup.bash`？
2. 可执行为什么装在 `lib/<pkg>/` 而不是 `bin/`？
3. `ament_export_targets` 和 `ament_export_dependencies` 各解决什么问题？
4. `ament_target_dependencies(node rclcpp)` 和 `target_link_libraries(node rclcpp::rclcpp)` 的区别？
5. `package.xml` 里的 `depend` 展开成哪三个标签？
6. `--symlink-install` 对 C++ 包和 Python 包分别意味着什么？
7. 两个工作空间都有 `robot_math`，`ros2 run` 跑的是哪个？怎么确认？
8. Python 包 `setup.cfg` 里的 `install_scripts` 为什么必要？
9. `install(TARGETS ...)` 和 `install(DIRECTORY ...)` 的区别？
10. 让 colcon 跳过某个目录，怎么做？

---

## 14. 验收清单

- [ ] 能脱稿画出并讲清五段链路，指出每段坏了会报什么错
- [ ] `colcon build --packages-up-to robot_math_demo` 成功，节点能跑、`/wheel_speed` 有数据
- [ ] 能说出 `robot_mathConfig.cmake` 里 `include(...)` 指向哪个文件、里面 `add_library(... IMPORTED)` 长什么样
- [ ] C-1 ~ C-4 的报错都能复现、能解释、能改回
- [ ] `ros2 pkg prefix` / `ldd` 能定位"这个包/这个库到底来自哪"
- [ ] 自检 10 题答对 ≥ 8

---

## 15. 延伸阅读

- 概念：<https://docs.ros.org/en/humble/Concepts/Basic/About-Packages.html>
- ament_cmake 用户文档：<https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Documentation.html>
- colcon 文档：<https://colcon.readthedocs.io/>
- 源码：`/opt/ros/humble/share/ament_cmake/cmake/ament_export_targets.cmake`（很短，值得读）

---

## 附：自检题答案

1. `setup.bash` 会连带 source 它记录的 underlay 链；`local_setup.bash` 只设当前 prefix。多 workspace 叠加用后者更可控。
2. `ros2 run` 的查找路径就是 `<prefix>/lib/<pkg>/<exe>`，这是 ROS 2 的目录约定；装到 `bin/` 就找不到。
3. `ament_export_targets` 导出 imported target（含 include、link、编译选项）；`ament_export_dependencies` 让消费端自动 `find_package` 我的依赖，避免缺头文件 / undefined reference。
4. 前者读目标包导出的 `<pkg>_INCLUDE_DIRS` / `<pkg>_LIBRARIES` / `<pkg>_TARGETS` 变量；后者用 imported target，interface 传递更完整，是现代推荐写法。
5. `build_depend` + `build_export_depend` + `exec_depend`。
6. C++：install 里是软链，但代码改了仍要重编；Python/launch/yaml：软链生效，改了立即生效，不用重编。
7. `AMENT_PREFIX_PATH` 中靠前的；`ros2 pkg prefix <pkg>` 查看实际路径，必要时 `which`/`ldd` 交叉确认。
8. 让 `console_scripts` 生成的启动脚本装到 `lib/<pkg>/`，否则 `ros2 run` 找不到。
9. 前者安装**构建产物**（可执行、库），可同时登记导出集；后者安装**源码树里的静态文件**（头文件、launch、config、模型）。
10. 在目标目录放一个空的 `COLCON_IGNORE` 文件。
