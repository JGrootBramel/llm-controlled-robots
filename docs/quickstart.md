# Quickstart Guide

1. Prerequisites
2. Build the Docker image
3. Start the container and get shell (or use Docker Compose for robot + remote)
4. Launch the Gazebo simulation

## 1. Prerequisites

### Windows
- Ubuntu-20.04+ on Windows Subsystem for Linux (WSL) (tested with 22.04)
- Docker Desktop
    - enable integration with my default WSL distro

Start WSL on Windows as root user: 
```bash
wsl -u root
```


### Mac

### Ubuntu 20.04 +
- python 3.10
- docker

## 2. Build  the docker Image

From Ubuntu CMD change clone this repository:
 ```bash
 git clone https://github.com/JGrootBramel/llm-controlled-robots.git
 ```

Change directory and build the Docker image:
 ```bash
 cd ~/llm-controlled-robots
 docker build -t noetic-gazebo-rosa -f ./sim/docker/Dockerfile .
 ```
Create a dot `.env` file containing your OPENAI_API_KEY. There is a `.example.env`file in the repository  for your referencce.
```bash
OPENAI_API_KEY="YOUR_KEY_HERE"
```

This command will build the container, give it 

## 3. Start the container and get shell

### Option A: Docker Compose (robot + remote)

Replicates the two-machine setup: one container runs the robot (Gazebo + bridge), the other runs the ROSA app and connects to the robot's ROS master.

From repo root:

```bash
cp .example.env .env
# Edit .env and set OPENAI_API_KEY

docker compose -f sim/docker/docker-compose.yml up --build
```

- **robot** service: Gazebo + limo_cobot_sim + limo_rosa_bridge.
- **remote** service: waits for roscore, then starts the ROSA chat (`python -m limo_llm_control.main`). Use the remote container for natural-language control.

To run with a visible Gazebo window, see the optional X11 settings in `sim/docker/docker-compose.yml` (uncomment DISPLAY and `/tmp/.X11-unix` for the robot service) and ensure `xhost +local:root` on Linux.

### Option B: Single container (legacy)

If that finishes, start the container:
 ```bash
docker run -it --rm --name limo_sim \
  --net=host \
  --env-file .env \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$(pwd)/src/tools:/src/tools" \
  noetic-gazebo-rosa
```

## 4. Launch the Gazebo simulation

Typical patterns are:

```bash
roslaunch <robot_package> <gazebo_launch>.launch
```
In a  new shell:

```bash
docker exec -it limo_sim bash
```

You can find some simple commands how to control the arm and the base seperately in this [simple controls examples](./simple_control_examples.md)  file.