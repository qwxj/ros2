
#!/usr/bin/env python3

import os

# 地图参数
RESOLUTION = 0.05

MAP_WIDTH = 10.0
MAP_HEIGHT = 8.0

ORIGIN_X = -5.0
ORIGIN_Y = -4.0

WIDTH = int(MAP_WIDTH / RESOLUTION)
HEIGHT = int(MAP_HEIGHT / RESOLUTION)

# 灰度值
FREE = 254
OCCUPIED = 0

# 地图中的矩形障碍物
# 格式：(xmin, ymin, xmax, ymax)
OBSTACLES = [
    (-2.5, -1.0, -1.8, 1.5),
    (-0.2, -3.0, 0.6, -1.0),
    (1.2, 0.5, 2.8, 1.2),
    (-0.5, 2.0, 0.8, 2.6),
]

# 创建全空地图
grid = [
    [FREE for _ in range(WIDTH)]
    for _ in range(HEIGHT)
]


def world_to_pixel(x, y):
    """世界坐标转换为PGM像素坐标"""

    col = int((x - ORIGIN_X) / RESOLUTION)
    row = HEIGHT - 1 - int((y - ORIGIN_Y) / RESOLUTION)

    return row, col


def draw_rectangle(xmin, ymin, xmax, ymax):
    """绘制矩形障碍物"""

    row1, col1 = world_to_pixel(xmin, ymax)
    row2, col2 = world_to_pixel(xmax, ymin)

    for row in range(row1, row2 + 1):
        for col in range(col1, col2 + 1):

            if 0 <= row < HEIGHT and 0 <= col < WIDTH:
                grid[row][col] = OCCUPIED


def main():

    # 四周墙壁，厚度约0.15m
    wall = 0.15

    # 下墙
    draw_rectangle(
        ORIGIN_X,
        ORIGIN_Y,
        ORIGIN_X + MAP_WIDTH,
        ORIGIN_Y + wall
    )

    # 上墙
    draw_rectangle(
        ORIGIN_X,
        ORIGIN_Y + MAP_HEIGHT - wall,
        ORIGIN_X + MAP_WIDTH,
        ORIGIN_Y + MAP_HEIGHT
    )

    # 左墙
    draw_rectangle(
        ORIGIN_X,
        ORIGIN_Y,
        ORIGIN_X + wall,
        ORIGIN_Y + MAP_HEIGHT
    )

    # 右墙
    draw_rectangle(
        ORIGIN_X + MAP_WIDTH - wall,
        ORIGIN_Y,
        ORIGIN_X + MAP_WIDTH,
        ORIGIN_Y + MAP_HEIGHT
    )

    # 内部障碍物
    for obstacle in OBSTACLES:
        draw_rectangle(*obstacle)

    output_dir = os.path.dirname(os.path.abspath(__file__))

    pgm_path = os.path.join(output_dir, "map.pgm")
    yaml_path = os.path.join(output_dir, "map.yaml")

    # 保存 PGM 文件
    with open(pgm_path, "w") as f:
        f.write("P2\n")
        f.write("# Generated occupancy grid map\n")
        f.write(f"{WIDTH} {HEIGHT}\n")
        f.write("255\n")

        for row in grid:
            f.write(" ".join(map(str, row)))
            f.write("\n")

    # 保存 Nav2 地图配置
    with open(yaml_path, "w") as f:
        f.write("image: map.pgm\n")
        f.write(f"resolution: {RESOLUTION}\n")
        f.write(f"origin: [{ORIGIN_X}, {ORIGIN_Y}, 0.0]\n")
        f.write("negate: 0\n")
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.196\n")
        f.write("mode: trinary\n")

    print("地图生成完成！")
    print(f"地图尺寸: {WIDTH} x {HEIGHT}")
    print(f"分辨率: {RESOLUTION} m/pixel")
    print(f"PGM: {pgm_path}")
    print(f"YAML: {yaml_path}")


if __name__ == "__main__":
    main()
