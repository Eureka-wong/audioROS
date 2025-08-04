FROM audio_ros:latest
# 安装GUI依赖
RUN apt-get update && apt-get install -y \
    python3-tk \
    xvfb \          
    x11-utils \      
    mesa-utils && \  
    rm -rf /var/lib/apt/lists/*


# RUN chmod +x /run.sh
# 保持TkAgg后端
