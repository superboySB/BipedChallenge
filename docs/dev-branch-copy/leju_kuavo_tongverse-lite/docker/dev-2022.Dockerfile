# FROM nvcr.io/nvidia/isaac-sim:2022.2.1-ubuntu20.04
FROM harbor.mybigai.ac.cn/tongverse/isaac-sim:2022.2.1-ubuntu20.04


RUN apt-get update \
    && apt-get install -y \
        ca-certificates gnupg lsb-release wget unzip \
    && wget -qO- https://drake-apt.csail.mit.edu/drake.asc | gpg --dearmor - > /etc/apt/trusted.gpg.d/drake.gpg \
    && echo "deb [arch=amd64] https://drake-apt.csail.mit.edu/$(lsb_release -cs) $(lsb_release -cs) main" > /etc/apt/sources.list.d/drake.list \
    && apt-get update \
    && DEBIAN_FRONTEND=noninteractive TZ=Asia/Shanghai apt-get install -y \
        libgflags2.2 \
        libsdl2-2.0-0 \
        libsdl2-ttf-2.0-0 \
        drake-dev \
    && rm -rf /var/lib/apt/lists/*

RUN cd / \
    && wget "http://10.2.31.187/tongverse-lite-leju/leju_controller-v1.zip" \
    && unzip leju_controller-v1.zip \
    && rm -rf leju_controller-v1.zip

RUN /isaac-sim/python.sh -m pip install --upgrade pip

WORKDIR /TongVerse/
ENTRYPOINT ["bash", "/TongVerse/docker/scripts/main-dev.sh"]
