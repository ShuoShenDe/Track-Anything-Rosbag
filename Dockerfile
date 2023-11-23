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
    curl\
    llvm \
    libncurses5-dev \
    libncursesw5-dev \
    xz-utils \
    tk-dev \
    libffi-dev \
    liblzma-dev \
    python3-openssl \
    build-essential \
    libssl-dev \
    zlib1g-dev \
    libreadline-dev \
    libsqlite3-dev \
    libgdbm-dev \
    libdb5.3-dev \
    libbz2-dev \
    libexpat1-dev \
    liblzma-dev \
    libffi-dev


RUN curl https://pyenv.run | bash

ENV PYENV_ROOT="/root/.pyenv"
ENV PATH="$PYENV_ROOT/shims:$PYENV_ROOT/bin:$PATH"
RUN echo "PATH is $PATH"

## Install Python 3.10
RUN pyenv install 3.10  && \
    pyenv global 3.10

RUN export PYTHON_VERSION="$(python --version | cut -d " " -f 2 | cut -d "." -f 1-2)" && \
    echo "PYTHON_VERSION is $PYTHON_VERSION"

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
RUN apt-get update && \
    pip install --extra-index-url https://rospypi.github.io/simple/ rospy rosbag roslz4 cv-bridge && \
    pip install --no-cache-dir -r requirements.txt && \
    echo "source /opt/ros/noetic/setup.bash" >> /etc/bash.bashrc


# 复制文件到容器中
COPY . .
RUN chmod +x /workspace/start_script.sh

# 设置当启动容器时运行的命令
ENTRYPOINT ["/workspace/start_script.sh"]
#CMD ["--path", "/workspace/shared_data/Compass_2D_demo_allTopic_with_sam.bag", "--frame_start_id", "0", "--mask", "/workspace/shared_data/frame1_mask.npy", "--frame_end_id", "None", "--mask_save_path", "/workspace/shared_data/masks_result.npy", "--topic", "CAM_FRONT"]
#CMD ["sh", "-c", "python sam_service.py"]