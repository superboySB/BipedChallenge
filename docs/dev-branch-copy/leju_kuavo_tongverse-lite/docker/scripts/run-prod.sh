xhost +
docker run --name tongverselite -it --gpus all -e "ACCEPT_EULA=Y" --rm --network=host \
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
  tongverselite
