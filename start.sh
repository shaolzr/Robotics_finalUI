#!/bin/bash

# 激活虚拟环境
source .venv/bin/activate

# 在虚拟环境中设置 ROS 环境
source /opt/ros/noetic/setup.bash
source /fetch_ws/devel/setup.bash

# 在虚拟环境中启动 ROS 服务（如果需要）
# 如果服务已经在运行，可以注释掉这行
# rosrun string_service_demo string_service_node.py &

# 在虚拟环境中运行主程序
python main.py

# 保持虚拟环境激活状态
exec $SHELL