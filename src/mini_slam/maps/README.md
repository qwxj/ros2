# maps/

放**包内地图**的目录。`map:=` 参数可以直接引用这里的文件名。

## 用法

1. 把你的地图拷进来（两个文件必须同名、放同一目录）：

```
cp ~/mini_map.yaml maps/my_map.yaml
cp ~/mini_map.pgm  maps/my_map.pgm
```

2. 确认 yaml 里的 `image:` 是**相对文件名**（不是绝对路径）：

```yaml
image: my_map.pgm        # ✅ 对
# image: /home/wxj/xxx.pgm   ❌ 错，不能跟 yaml 一起搬
```

3. 重新编译（让 cmake 把新文件装到 share/）：

```bash
colcon build --packages-select mini_slam
```

4. 按名字引用：

```bash
ros2 launch mini_slam debug_map.launch.py map:=my_map
ros2 launch mini_slam amcl_mini.launch.py map:=my_map
```

## 为什么不放示例地图

示例地图是从某次调试里跑出来的**局部地图**，换场地就没意义了。
真要用，请用 `slam_mini_node` 建一张自己的。
