# CRAIC2024
人形机器人创新挑战赛  BIT-LINC队伍方案

## 安装指南
下载官方镜像并且构建容器
```sh
wget https://roban.lejurobot.com/humanoid-tongverselite/tongverselite-release-docker_20240104.tar.gz

docker load -i tongverselite-release-docker_20240104.tar.gz

xhost +

docker run --name tongverselite-release -itd --gpus all -e "ACCEPT_EULA=Y" --rm --network=host \
  --ulimit rtprio=99 -e "PRIVACY_CONSENT=Y" --privileged \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro -e DISPLAY=$DISPLAY \
  -v ~/docker/isaac-sim_2023.1.0/cache/kit:/isaac-sim/kit/cache:rw \
  -v ~/docker/isaac-sim_2023.1.0/cache/ov:/root/.cache/ov:rw \
  -v ~/docker/isaac-sim_2023.1.0/cache/pip:/root/.cache/pip:rw \
  -v ~/docker/isaac-sim_2023.1.0/cache/glcache:/root/.cache/nvidia/GLCache:rw \
  -v ~/docker/isaac-sim_2023.1.0/cache/computecache:/root/.nv/ComputeCache:rw \
  -v ~/docker/isaac-sim_2023.1.0/logs:/root/.nvidia-omniverse/logs:rw \
  -v ~/docker/isaac-sim_2023.1.0/data:/root/.local/share/ov/data:rw \
  -v ~/docker/isaac-sim_2023.1.0/documents:/root/Documents:rw \
  tongverselite-release:v1.0 /bin/bash
```
先尝试用虚拟机的思路来使用容器
```sh
rm -rf /BipedChallenge && cd /

git clone https://github.com/superboySB/CRAIC2024

mv /CRAIC2024 /BipedChallenge
```
