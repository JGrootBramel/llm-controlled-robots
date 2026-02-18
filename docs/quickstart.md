# Quickstart Guide

1. Prerequisit
2. Build the docker Image
3. Start the container and get shell
4. Launch the Gazebo simulation

## 1. Prerequisits

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

Change directory and bulid the docker container:
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