# 使用官方的NVIDIA CUDA基础镜像
FROM nvidia/cuda:12.2.2-cudnn8-runtime-ubuntu20.04

# 设置时区，避免构建时询问
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Berlin

# 安装基础依赖
RUN apt-get update && apt-get install -y \
    software-properties-common \
    build-essential \
    cmake \
    wget \
    git \
    curl

# 安装ROS Noetic及其依赖
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update && apt-get install -y ros-noetic-desktop-full python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential && \
    apt-get install -y --no-install-recommends python3-rosinstall python3-rosinstall-generator python3-wstool build-essential && apt-get install -y ros-noetic-cv-bridge ros-noetic-sensor-msgs &&\
    rosdep init && rosdep update && \
    rm -rf /var/lib/apt/lists/*

# 配置工作目录
WORKDIR /workspace

COPY requirements.txt .

# 安装Python依赖
RUN apt-get update && apt-get install -y python3-pip && \
    pip3 install --upgrade pip && \
    pip3 install --extra-index-url https://rospypi.github.io/simple/ rospy rosbag roslz4 cv-bridge && \
    pip3 install --no-cache-dir -r requirements.txt

# 复制文件到容器中
COPY . .

# 设置当启动容器时运行的命令
ENTRYPOINT ["/workspace/start_script.sh"]
CMD ["--path", "/workspace/shared_data/Compass_2D_demo_allTopic_with_sam.bag", "--frame_start_id", "0", "--mask", "/workspace/shared_data/frame1_mask.npy", "--frame_end_id", "None", "--mask_save_path", "/workspace/shared_data/masks_result.npy", "--topic", "CAM_FRONT"]
