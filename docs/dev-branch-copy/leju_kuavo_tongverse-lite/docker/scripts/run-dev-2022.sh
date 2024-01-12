SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
PROJECT_DIR=$(realpath "$SCRIPT_DIR/../../")

xhost +
docker run --name tongverselite-dev-2022 -it --gpus all -e "ACCEPT_EULA=Y" --rm --network=host \
  -e "PRIVACY_CONSENT=Y" \
  -v $HOME/.Xauthority:/root/.Xauthority \
  -e DISPLAY \
  -v ~/docker/isaac-sim_2022.2.1/cache/kit:/isaac-sim/kit/cache:rw \
  -v ~/docker/isaac-sim_2022.2.1/cache/ov:/root/.cache/ov:rw \
  -v ~/docker/isaac-sim_2022.2.1/cache/pip:/root/.cache/pip:rw \
  -v ~/docker/isaac-sim_2022.2.1/cache/glcache:/root/.cache/nvidia/GLCache:rw \
  -v ~/docker/isaac-sim_2022.2.1/cache/computecache:/root/.nv/ComputeCache:rw \
  -v ~/docker/isaac-sim_2022.2.1/logs:/root/.nvidia-omniverse/logs:rw \
  -v ~/docker/isaac-sim_2022.2.1/data:/root/.local/share/ov/data:rw \
  -v ~/docker/isaac-sim_2022.2.1/documents:/root/Documents:rw \
  -v $PROJECT_DIR:/TongVerse:rw \
  tongverselite-dev-2022 bash
