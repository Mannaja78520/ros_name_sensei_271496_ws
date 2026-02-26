FROM ubuntu:24.04

# 1. ตั้งค่า Locale
RUN apt-get update && apt-get install -y locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# 2. ติดตั้ง System Dependencies
RUN apt-get update && apt-get install -y \
    software-properties-common curl git python3-pip \
    libgl1 libglib2.0-0 gnupg2 lsb-release \
    && add-apt-repository universe -y

# 3. ติดตั้ง ROS 2 Jazzy
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys F42ED6FBAB17C654 \
    && echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu noble main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update && apt-get install -y \
    ros-jazzy-ros-base ros-dev-tools

# 4. สร้าง Workspace และ Copy ไฟล์ (จะใช้ไฟล์ .dockerignore กรองไฟล์ที่ไม่เอาออก)
WORKDIR /ros_ws
COPY . .

# แก้ไข Permission
RUN chmod +x 660610840_final/src/name_sensei_proj/control/mediapipe/*.py \
             660610840_final/src/name_sensei_proj/src/*.py

# 5. ติดตั้ง Python Packages ลงในระบบ
# ใช้ --ignore-installed เพื่อข้ามปัญหา Package ที่ซ้ำกับระบบ (เช่น PyYAML)
RUN if [ -f requirements.txt ]; then \
        pip install --no-cache-dir -r requirements.txt --ignore-installed --break-system-packages; \
    fi

# 6. Build ROS 2
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install

# 7. ตั้งค่า bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source /ros_ws/install/setup.bash" >> ~/.bashrc && \
    echo "export ROS_DOMAIN_ID=1" >> ~/.bashrc 

CMD ["bash"]