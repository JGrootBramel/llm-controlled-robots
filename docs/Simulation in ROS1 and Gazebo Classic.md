# Simulating Limo Cobot in ROS1 and Gazebo Classic

For testing we try to generate a dockerfile with the necessery Ubuntu version and necessary software installed.

You can find the Dockerfile in here: [Dockerfile.noetic](../Dockerfile.noetic)

## Prerequisits

Linux
Docker installed from [Docker.com]

## Build Dockerfile
On linux from the root of this project execute:

```bash
docker build -t noetic-gazebo-rosa -f ./Dockerfile.noetic .

```

```bash
xhost +local:root
docker run -it --rm \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  noetic-gazebo-rosa
```
