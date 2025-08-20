#!/bin/bash
set -e  # 任何命令失败时立即退出

# 加载 ROS 环境
source /opt/ros/galactic/setup.bash

# 设置Python路径
export PYTHONPATH="/audioROS/crazyflie-audio/python:$PYTHONPATH"
export PYTHONPATH="/audioROS/python:$PYTHONPATH"
export PYTHONPATH="/audioROS/python/utils:$PYTHONPATH"

# 清理之前的编译文件（可选）
rm -rf build install log

# 清理可能的缓存
rm -rf ~/.colcon_cache 2>/dev/null || true

# 清理可能冲突的Python缓存
find . -name "__pycache__" -type d -exec rm -rf {} + 2>/dev/null || true
find . -name "*.pyc" -delete 2>/dev/null || true

# 清理特定包的符号链接冲突
echo "Cleaning potential symlink conflicts..."
rm -rf build/crazyflie_description/ament_cmake_python 2>/dev/null || true
rm -rf build/audio_interfaces/ament_cmake_python 2>/dev/null || true

# 编译所有需要的包
echo "Building audio_interfaces package..."
colcon build --packages-select audio_interfaces --cmake-args -DCMAKE_BUILD_TYPE=Release

# 编译crazyflie_description包（不使用symlink）
echo "Building crazyflie_description package..."
colcon build --packages-select crazyflie_description --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "Building audio_bringup package..."
colcon build --packages-select audio_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "Building audio_stack package..."
colcon build --packages-select audio_stack --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "Building all remaining packages..."
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release


# 加载工作区环境
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "Sourcing workspace setup..." 
else
    echo "ERROR: install/setup.bash not found. Build may have failed."
    exit 1
fi

echo "Setting up Python paths..."
export PYTHONPATH="/audioROS/crazyflie-audio/python:$PYTHONPATH"
export PYTHONPATH="/audioROS/python:$PYTHONPATH"
export PYTHONPATH="/audioROS/python/utils:$PYTHONPATH"
export PYTHONPATH="/audioROS:$PYTHONPATH"
# 验证包是否可用
# echo "Checking if audio_bringup package is available..."
# ros2 pkg list | grep audio_bringup || {
#     echo "ERROR: audio_bringup package not found after build"
#     exit 1
# }

# # 列出可用的launch文件
# echo "Available launch files:"
# find install -name "*.launch.py" | grep audio_bringup || echo "No launch files found"

# # 启动主程序（取消注释您想要运行的命令）
# echo "Starting ROS2 launch..."
# exec xvfb-run -a ros2 launch audio_bringup doa_simulated.launch.py

# 其他可用的启动选项：
# exec xvfb-run -a ros2 launch audio_bringup doa_real.launch.py
# exec xvfb-run -a ros2 launch audio_bringup wall_simulated.launch.py
# exec xvfb-run -a ros2 launch audio_bringup live_demo.launch.py