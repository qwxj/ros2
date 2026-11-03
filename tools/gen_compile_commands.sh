#!/usr/bin/env bash
# 重新生成 compile_commands.json（VS Code C++ 补全用）
#
# 什么时候要跑：
#   1. 新建了源文件 / 新包之后
#   2. VS Code 补全突然大面积标红
#
# 用法：
#   bash tools/gen_compile_commands.sh
#   然后在 VS Code 里按 Ctrl+Shift+P -> "Reload Window"

set -e
cd "$(dirname "$0")/.."
WORKSPACE="$(pwd)"
echo "[1/3] 带 -DCMAKE_EXPORT_COMPILE_COMMANDS=ON 重新编译..."
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON "$@"

echo "[2/3] 合并各包的 compile_commands.json ..."
python3 - <<'PY'
import json, glob, os
root = os.getcwd()
merged, seen = [], set()
files = glob.glob(os.path.join(root, "build", "*", "compile_commands.json"))
for f in files:
    try:
        data = json.load(open(f, encoding="utf-8"))
    except Exception as e:
        print("  跳过 %s: %s" % (f, e)); continue
    for entry in data:
        key = entry.get("file")
        if key and key not in seen:
            seen.add(key)
            merged.append(entry)
out = os.path.join(root, "compile_commands.json")
json.dump(merged, open(out, "w", encoding="utf-8"), indent=1)
print("  合并 %d 个包 -> %s（%d 条编译单元）" % (len(files), out, len(merged)))
PY

echo "[3/3] 完成。"
echo "  现在在 VS Code 里按 Ctrl+Shift+P -> Reload Window"
echo "  或者 Ctrl+Shift+P -> C/C++: Reset IntelliSense Database"