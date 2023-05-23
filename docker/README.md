# PILOTING gRCS Docker image
---

**In case you have a nvidia graphics card, it is recommended to have the drivers installed for a better performance of the application.**


## Tested Systems
---

* Ubuntu 18.04/20.04 LTS with and without Nvidia drivers
* Windows 10 (Unstable)

# Ubuntu System

## Docker Image creation
---

In order to create the gRCS docker image, you need to run the following script:
```
./create_grcs_image.sh
```

## Installation and usage on Ubuntu
---

First of all, docker must be installed on the system. To do this, use the following script:

```
bash install_docker.sh
```

After this, open a new terminal. If you have the nvidia drivers installed, use the following script:

```
bash run_grcs.sh --nvidia_drivers=True
```

Otherwise, the input parameter must be changed:

```
bash run_grcs.sh --nvidia_drivers=False
```

## Troubleshooting on Ubuntu
---

### Docker daemon permissions

```
docker: Got permission denied while trying to connect to the Docker daemon socket at unix:///var/run/docker.sock: Post "http://%2Fvar%2Frun%2Fdocker.sock/v1.24/containers/create": dial unix /var/run/docker.sock: connect: permission denied.
See 'docker run --help'.
```
The following command should solve the problem:
```
sudo chmod 666 /var/run/docker.sock
```
### X Display authorization error

```
Authorization required, but no authorization protocol specified
qt.qpa.screen: QXcbConnection: Could not connect to display :0
Could not connect to any X display.
```

The following command should solve the problem:
```
xhost +local:docker
```

### Unable to Ogre rendering window

```
OGRE EXCEPTION(3:RenderingAPIException): Unable to create a suitable GLXContext in GLXContext::GLXContext at /build/ogre-1.9-B6QkmW/ogre-1.9-1.9.0+dfsg1/RenderSystems/GL/src/GLX/OgreGLXContext.cpp (line 61)
```

This problem is usually caused by launching the container without gpu but having the nvidia drivers installed. You should try to launch the script with the "nvidia_drivers=True" parameter.

### Problems with xauth

```
docker: Error response from daemon: OCI runtime create failed: container_linux.go:380: starting container process caused: process_linux.go:545: container init caused: rootfs_linux.go:76: mounting "/tmp/.docker.xauth" to rootfs at "/tmp/.docker.xauth" caused: mount through procfd: not a directory: unknown: Are you trying to mount a directory onto a file (or vice-versa)? Check if the specified host path exists and is the expected type.
```

A possible solution is to comment out the  function *"configure_xserver"* from the  script *"run_grcs.sh"*.

# Windows System

On Windows, there is a problem with the use of OpenGL over WSL, which is used by the Docker client.
That is the reason why the map display is slow.

We are working on fixing this bug. 

## Installation and usage on Windows
---

### Installation 
First of all, you must install *Docker Desktop* on Windows following this documentation: https://docs.docker.com/desktop/windows/install/

Afterwards you must install *Windows X Server* with this link: https://sourceforge.net/projects/vcxsrv/

### Usage

Restart the computer. After startup, run the XLaunch program and follow the default options until *Extra Settings* window. There, uncheck *Native opengl* option. This will activate the X Server to 
enable screen sharing between the container and the computer.

Next, you need to know your IP address. To do this, open a *PowerShell terminal* and run the following command. Search the command output for your WSL IPv4 address.
```
ipconfig
```

Finally, navigate to the folder where the docker image is and open a *Powershell terminal* in that directory. Run the following commands: (Change *<your-ip>* to the IP obtained in the above command)
```
set-variable -name DISPLAY -value <your-ip>:0.0

docker load -i .\piloting_grcs_latest.tar.gz

docker container run -it --rm --security-opt apparmor:unconfined --ipc host --network host -e "DISPLAY=$DISPLAY" -e QT_X11_NO_MITSHM=1 catecupia/piloting_grcs
```

## Troubleshooting on Windows
---

### OpenGL error

If you get the following error when running gRCS on Windows, make sure that when you ran *XLaunch* you had unchecked *Native opengl*.
```
libGL error: No matching fbConfigs or visuals found
libGL error: failed to load driver: swrast
/ros_entrypoint.sh: line 16:    61 Segmentation fault      piloting_grcs_node
```