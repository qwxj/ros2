# ros2_control 接口与插件链路

> 对应代码：`src/my_robot_control/`（`diff_drive_hardware.cpp`、`imu_sensor.cpp`、`my_robot_hardware_plugin.xml`）

## 1. 三层关系

```
URDF 的 <ros2_control>        声明"这个硬件对外提供哪些接口"
        ↕ 按名字匹配
插件的 export_*_interfaces()  实现：把 double* 交出去
        ↕ 按名字取
控制器 / broadcaster          消费这些接口
```

框架（`resource_manager`）在中间做核对：声明了但没导出 → configure 失败；导出了但没声明 → 不生效。

## 2. 三种 type

| type | 基类 | 粒度 | 本项目 |
|---|---|---|---|
| `system` | `SystemInterface` | 多关节，一次 read/write 全读全写 | 差速底盘（左右轮） |
| `actuator` | `ActuatorInterface` | 单关节 | 未使用 |
| `sensor` | `SensorInterface` | 只读，无 command | IMU |

`type` 必须与插件继承的基类一致，否则 pluginlib 加载时类型转换失败。

## 3. 从源码到运行：六个环节

1. `CMakeLists.txt` 的 `add_library` 生成 `libmy_robot_control.so`；
2. `PLUGINLIB_EXPORT_CLASS` 在 .so 内登记「类名 → 工厂」；
3. `my_robot_hardware_plugin.xml` 记录 `library / class name / type / base_class_type`；
4. `pluginlib_export_plugin_description_file(hardware_interface ...)` 把 xml 装进 share 并写入 ament index；
5. URDF 的 `<plugin>` 写插件名，`resource_manager` 用 `ClassLoader` 查索引、`dlopen`、`new`；
6. 按生命周期调用：`on_init` → `export_*` → `on_configure` → `on_activate` → 100 Hz `read/write`。

五处名字必须一致：`add_library` ↔ `library path`，宏里的类 ↔ xml `type`，基类 ↔ `base_class_type`，xml `name` ↔ URDF `<plugin>`，注册时的包名 ↔ `ClassLoader` 第一个参数。

## 4. 缓冲分配规则

> 必须在 `export_*_interfaces()` 之前分配好，之后**永远不改容量**。

- 底盘：关节数由 URDF 决定，只能在 `on_init` 里 `assign`；
- IMU：10 个字段固定，在成员初始化列表里定长（`orientation_{0,0,0,1}`）。

`on_configure` / `on_cleanup` 里只能 `fill`，不能 `assign`/`clear`/`operator=`——重新分配会让框架持有的指针悬空，表现为 `get_value() failed: value_ptr_ is null`，随后进程 SIGABRT。

## 5. read / write

```
controller_manager 实时循环（update_rate = 100 Hz）：
  read()            → 填 state 指针
  controller update → 读 state、写 command 指针
  write()           → 把 command 发给硬件
```

电机模型（`write()`）：

- 速度模式：`τ = b·ω_target + Kp·(ω_target − ω)`（摩擦前馈 + 比例反馈）
- 力矩模式：`τ = τ_cmd`（开环）
- 物理：`dω/dt = (τ − b·ω) / J`

三处限幅：指令限速、力矩限幅、积分后限速。

编码器量化（`read()`）：`res = 2π/4096 ≈ 1.5e-3 rad`；低速时一个周期走不满一格会出现位置台阶；位置差分算速度的噪声约为 `res/(2·dt) ≈ ±0.077 rad/s`（100 Hz）。用 `position_raw_` 连续积分、每周期重新量化，可让误差有界不累积。

## 6. 换真实硬件改哪里

- 插件内部的 `read()/write()`：从模拟电机换成串口/CAN 协议帧；
- 新增参数：设备路径、波特率、减速比、轮径、编码器线数；
- URDF 的 `<plugin>` 与 `<param>`；
- **接口名与单位不变**（position/velocity/effort、rad、m/s）→ `diff_drive_controller`、Nav2、EKF 无需改动。
