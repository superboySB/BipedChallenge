# CRAIC2024
人形机器人创新挑战赛  BIT-LINC队伍方案

## 安装指南
下载官方镜像并且构建容器
```sh
wget https://roban.lejurobot.com/humanoid-tongverselite/tongverselite-release-docker_20240104.tar.gz

md5sum -c checksum.txt

docker load -i tongverselite-release-docker_20240104.tar.gz

xhost +

docker run --name tongverselite-release -itd --gpus all -e "ACCEPT_EULA=Y" --network=host \
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
docker exec -it tongverselite-release /bin/bash

apt-get update && apt-get install git dos2unix vulkan-tools

cd / && rm -rf /BipedChallenge

git clone https://github.com/superboySB/BipedChallenge && cd /BipedChallenge
```
尝试运行demo，task从1-6指定。
```sh
bash examples/launch_task.sh <task-id>
```
如果出现转码问题，可以用`dos2unix examples/launch_task.sh`转换编码。因为我暂时没有ubuntu台式机，我选择用headless模式运行脚本，然后本机接steaming client，这个做法如果有台式机可以去掉`--no-window`
