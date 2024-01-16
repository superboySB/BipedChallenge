# CRAIC2024
人形机器人创新挑战赛  BIT-LINC队伍方案

## 安装指南
下载官方镜像并且构建容器
```sh
wget https://roban.lejurobot.com/humanoid-tongverselite/tongverselite-release-docker_20240104.tar.gz

md5sum -c checksum.txt

docker load -i tongverselite-release-docker_20240104.tar.gz

bash docker-run-release.sh
```
如果出现转码问题，可以用`dos2unix docker-run-release.sh`转换编码,先尝试用虚拟机的思路来使用容器
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
