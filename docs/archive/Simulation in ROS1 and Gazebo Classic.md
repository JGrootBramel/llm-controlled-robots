# Simulating Limo Cobot in ROS1 and Gazebo Classic

For testing we try to generate a dockerfile with the necessery Ubuntu version and necessary software installed.

You can find the Dockerfile in here: [sim/docker/Dockerfile](../sim/docker/Dockerfile)

## Prerequisits

Linux
Docker installed from [Docker.com]

## Build Dockerfile
On linux from the root of this project execute:

```bash
docker build -t noetic-gazebo-rosa -f ./sim/docker/Dockerfile .

```

Run the simulation:
```bash
xhost +local:root
docker run -it --rm \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  noetic-gazebo-rosa
```
