# hdl-localization-lidar-camera-calibration

### build docker container
```
nvidia-docker run --privileged -it \
           -e NVIDIA_DRIVER_CAPABILITIES=all \
           -e NVIDIA_VISIBLE_DEVICES=all \
           --volume=/home/husky/Desktop/5gamm_original:/home/hdl-localization-ros1 \
           --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
           --net=host \
           --ipc=host \
           --device=/dev/ttyUSB0:/dev/ttyUSB0 \
           --shm-size=2gb \
           --name= hdl-localization-ros1 \
           --env="DISPLAY=$DISPLAY" \
           stereolabs/zed:3.7-gl-devel-cuda11.4-ubuntu20.04
```
