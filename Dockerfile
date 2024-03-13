FROM superboysb/tongverselite-release:v1.0

# Please contact with me if you have problems
LABEL maintainer="Zipeng Dai <daizipeng@bit.edu.cn>"

RUN apt-get update && \
    apt-get install -y --no-install-recommends locales git tmux gedit vim openmpi-bin openmpi-common libopenmpi-dev libgl1-mesa-glx \
    dos2unix vulkan-tools

WORKDIR /workspace/
RUN git clone https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs
RUN cd OmniIsaacGymEnvs && git checkout release/2023.1.0 && /isaac-sim/python.sh -m pip install --upgrade pip && \
    /isaac-sim/python.sh -m pip install -e .

# Yolo-world
WORKDIR /workspace/
RUN git clone https://github.com/superboySB/YOLOv8-TensorRT.git
RUN cd YOLOv8-TensorRT && \
    /isaac-sim/python.sh -m pip install -r requirements.txt && \
    /isaac-sim/python.sh -m pip install opencv-python==4.8.0.74 opencv-contrib-python==4.8.0.74 && \
    wget https://github.com/ultralytics/assets/releases/download/v8.1.0/yolov8l-world.pt
RUN cd YOLOv8-TensorRT && /isaac-sim/python.sh -m pip install git+https://github.com/openai/CLIP.git && \
    /isaac-sim/python.sh test_yoloworld.py


# EfficientViT + SAM
WORKDIR /workspace/
RUN git clone https://github.com/superboySB/efficientvit.git
RUN cd efficientvit && \
    /isaac-sim/python.sh -m pip install einops tqdm transformers pycocotools lvis mpi4py timm onnxruntime && \
    # /isaac-sim/python.sh -m pip install git+https://github.com/superboySB/torchprofile && \
    /isaac-sim/python.sh -m pip install git+https://github.com/superboySB/torchpack && \
    /isaac-sim/python.sh -m pip install git+https://github.com/alibaba/TinyNeuralNetwork.git && \
    /isaac-sim/python.sh -m pip install git+https://github.com/facebookresearch/segment-anything.git && \
    mkdir -p assets/checkpoints/sam && cd assets/checkpoints/sam && \
    wget https://huggingface.co/han-cai/efficientvit-sam/resolve/main/l2.pt && \
    wget https://huggingface.co/han-cai/efficientvit-sam/resolve/main/xl1.pt
# RUN cd efficientvit && /isaac-sim/python.sh demo_sam_model.py --model xl1 --mode point