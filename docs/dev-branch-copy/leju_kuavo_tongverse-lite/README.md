# TongVerse Lite 轻量版

## 要求

Docker + [Nvidia 容器工具包](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

## 开发设置

### Docker 设置

下载镜像, 地址： https://roban.lejurobot.com/humanoid-tongverselite/tongverselite-dev_docker.tar.gz

加载 docker 镜像：
```bash
docker load -i tongverselite-dev_docker.tar.gz
```

运行开发 docker 镜像：
```
bash docker/scripts/run-dev.sh
```
它将自动绑定（不是添加）您的本地仓库目录到 docker 中的 `/TongVerse/` 并启动一个新的 bash shell。

## 执行比赛任务脚本

启动 Docker 环境后，使用以下命令来运行任务：

```bash
bash examples/launch_task.sh <task-id>
```

将 `<task-id>` 替换为从 1 到 6 的整数，表示所需的任务。
