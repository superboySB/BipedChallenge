# CRAIC2024
人形机器人创新挑战赛  BIT-LINC队伍方案

## 安装指南
下载官方镜像并且构建容器,如果使用windows的话注意不要给docker开WSL2 backend，可能会有vulkan的问题
```sh
git clone https://github.com/superboySB/BipedChallenge && cd ./BipedChallenge

wget https://roban.lejurobot.com/humanoid-tongverselite/tongverselite-release-docker_20240104.tar.gz

md5sum -c checksum.txt

docker load -i tongverselite-release-docker_20240104.tar.gz

bash docker-run-release.sh
```
如果出现转码问题，可以用`dos2unix docker-run-release.sh`转换编码,先尝试用虚拟机的思路来使用容器
```sh
docker exec -it tongverselite-release /bin/bash

apt-get update && apt-get install git gedit dos2unix vulkan-tools
```
尝试运行demo，task从1-6指定。
```sh
bash examples/launch_task.sh <task-id>
```
如果出现转码问题，可以用`dos2unix examples/launch_task.sh`转换编码。注意目前这个工程暂时不支持headless模式（大概率是学艺不精），亲测需要使用ubuntu+RTX的台式机（需要插屏幕）来运行代码,否则会有奇怪问题。如果有类似`Failed to acquire IWindowing interface`的错误可以设置`export DISPLAY=:0`
