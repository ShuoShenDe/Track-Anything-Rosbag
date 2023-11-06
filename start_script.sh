#!/bin/bash

# 设置 ROS 环境
source /opt/ros/noetic/setup.bash

# 运行 Python 程序并传递所有参数
exec python3 start_tracking.py "$@"
