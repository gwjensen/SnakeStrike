---
declaration: "Environment and Dependencies"
title: "  Environment and Dependencies"
tags:
  - usage_page
layout: usage_page
---

## **Hardware Needed**

Obviously there are some basic hardware requirements for using this software as the software won't capture beams of light and turn them into
images all by itself!

The following are the recommended pieces of hardware:
#### **Cameras and Lenses**
 You need a minimum of two cameras with adequate lenses for your task for triangulation. They should also be of the same make/model to make your life easier.
   - I use Basler Ace ac1300-200uc USB 3.0 cameras, and consequently SnakeStrike will support any Basler USB connected camera without the need for a plugin. If you use other cameras you will need to write a plugin so that the cameras can interface with SnakeStrike.

#### **Triggering System**
 A triggering system for sending TTL pulses to the cameras.
   - I made my own using a DAQ from Labjack, the instructions are [Here - Follow me!](https://gwjensen.github.io/SnakeStrike/usage/recording/#homemade-ttl)

#### **Cables**
Cables to connect your camera to the computer and to connect your triggering system to your cameras.

#### **Sufficient Connection Ports**
Make sure there are enough USB 3 ports, or whatever ports your cameras require, to run the number of cameras you want. In the case of USB ports, just having them available on the housing of the computer is
not enough. You will want to make sure each camera has its own channel and controller. Typically, on a desktop computer the ports at the front of the
computer all share the same channel and controller, and the ports at the back all share the same channel and controller. In end effect, this means that
the frame rate of the camera might not be limited by the hardware of the camera,
but rather by the channel bandwidth.

For example, two cameras running 640 x 480 resolution at 750 Hz might not be able to achieve
full 750Hz without dropped frames if they are sharing the same channel as the data is being generated too fast and is fully saturating
the bandwidth. If you want to have more than two cameras running, I would suggest buying a PCI card to expand your ports.
   -I use the 4 port PCI express USB 3.0 card from Startech because it has 4 dedicated channels.

#### **Computer**
You absolutely need a beefy computer with multiple cores and a large amount of RAM and storage. I recommend a minimum of 64 GB of RAM and several TBs of storage.

The main reason is that each camera generates a lot of data. With 5 cameras running at 750Hz for 10 seconds at a resolution of 640 x 480 pixels the images by themselves
would be ~32 GBs of data. As the images captured by the cameras are buffered in memory before writting to disk, a large
amount of memory is needed. This ~32 GB of data need not be that large when stored to disk as it can be compressed
by saving it into a video, but the original capture images still need to be stored to buffer, otherwise images will be dropped.
The only time this isn't is the case is if the camera has built-in memory on board. If this is the case, however, we are
no longer talking about reasonably priced commodity cameras.


## **Compiling and/or running the SnakeStrike**

{% include note.html content="This code will only work in linux and has only been tested on Ubuntu 16.04." %}

Follow these instructions to build the tracking code from the github source. If you are happy to use the version of
SnakeStrike installed in the docker image you don't need to rebuild. Continue reading [Here - Follow me!](https://gwjensen.github.io/SnakeStrike/usage/setup/#running-the-snakestrike-gui)

### **1. Dependencies**
You have two options to solve the library dependencies of this program.

#### **Docker environment**
There is another github repository that contains the code for building a docker image. This docker image installs all of the dependencies and gives you an ideal build environment. This is the recommended route.

[The information for the docker is located here.](https://github.com/gwjensen/SnakeStrikeDocker)

{% include note.html content="This docker image is quite large after it is built. It is also a multi-stage build and needs roughly 70GB of free space for docker to complete the build. The final image is ~24 GB." %}

#### **User environment**
If using the docker environment is not an option for you, then you will need to load the required dependencies manually.<br> <br>
The best place to start with this is to pull the code from the docker repository and to follow the 
build steps inside the docker file. This ensures you are using the correct version of code as you will be compiling everything from source, but you also benefit from using the cmake init files that are included in the repository. <br> <br>
It is highly recommended to start with the cmake init files as the dependency chain is quite long. It sucks a lot to have forgotten a flag 5 libraries back and now you need to recompile large things like opencv again.

### **2. Download repository**

First, download or clone the code from the [Github repo](https://github.com/gwjensen/SnakeStrike).
<br>
```bash
git clone https://github.com/gwjensen/SnakeStrike
```

For the docker repository,<br>
```bash
git clone https://github.com/gwjensen/SnakeStrikeDocker
cd SnakeStrikeDocker
git submodules update --init
git submodules update
```

### **3. Building Docker image to use as runtime environment**
{% include note.html content="You can install docker by following the instructions here: [Install Docker-CE](https://docs.docker.com/install/linux/docker-ce/ubuntu/)." %}


#### **Creating the docker image**
```bash
cd <path_to_docker_repo>
docker build --tag=snakestrike-env --build-arg NUM_THREADS_FOR_BUILDS=XX
```
where XX is the number of processes that a call to ``` make -jXX ``` would expect. In other words, if you want this to build quicker, give it a number greater than 2, but less than the number of cores of your system. If
your system has 8 CPU cores, and you say 9, then the availability of your system will be greatly reduced while the image is compiling.
{% include note.html content="This is entirely up to your preferences. If it is empty it will use as many processes as it can. A good rule of thumb to balance speed and useability is to find the number of cpu cores your system has <br>
e.g. by running 
```bash
nproc --all 
``` 
or 
```bash 
grep processor /proc/cpuinfo | wc -l 
```
and subtracting one or more from this number." %}


#### **Making sure we can pass container windows correctly.**
Since this is a gui application, it doesn't make much sense to try to run it from docker if we can't see the windows.
<br> 
By running 
``` bash
sudo xhost +local:docker 
``` 
we are giving the docker process access to our local xwindow host and now we can forward windows created inside the docker container to our monitor.


### **Rebuilding tracker code inside container**
```bash
docker run -ti -v path_to_tos/tos:'/home/tos' -v '/dev/bus/usb':'/dev/bus/usb' --privileged -e DISPLAY=$DISPLAY -v '/tmp/.X11-unix':'/tmp/.X11-unix' snakestrike-env /bin/bash
```
This command maps the tracker code into the container as well as makes sure that any usb connected cameras will be found and any windows created inside the docker container will be forwarded. And finally, it opens a shell inside the container.
<br> <br>
From this shell inside the docker container, run the following commands:
```bash
cd /home/tos/build
cmake ..
sudo make install
```
<h4 id="commit-a-container">Committing a changed container</h4>
This step is not needed by everyone since the code that is being built resides outside of docker ( if the instructions were followed ;) ). The code doesn't
dissapear when the container dissapears. If your container exits and you need to run SnakeStrike again, you just need to run the following:

```bash
cd /home/tos/build
sudo make install
```

You don't need to run cmake again as the files still exist from before.

If, however, you have made changes to SnakeStrike and want to share those changes with someone else very easily, then you can save those changes to a new
docker image.

To save the changes you made in a new docker image, follow the example below:<br>
<div class="language-bash highlighter-rouge"><div class="highlight"><pre class="highlight"><code><span class="nv">$ </span>docker ps

CONTAINER ID     IMAGE            COMMAND       CREATED        STATUS        PORTS      NAMES
c3f279d17e0a     snakestrike-env  /bin/bash     7 days ago     Up 25 hours            desperate_dubinsky
197387f1b436     ubuntu:16.04     /bin/bash     7 days ago     Up 25 hours            focused_hamilton

<span class="nv">$ </span>docker commit c3f279d17e0a  snakestrike-env:version2

f5283438590d

<span class="nv">$ </span>docker images

REPOSITORY                        TAG                 ID                  CREATED             SIZE
snakestrike-env                 version2            f5283438590d        16 seconds ago      24.6GB
</code></pre></div></div>

## **Running the SnakeStrike gui**
If you don't need to re-build the SnakeStrike code, then after a successful build of the docker image, run the following command:
```bash
docker run -ti -v '/dev/bus/usb':'/dev/bus/usb' --privileged -e DISPLAY=$DISPLAY -v '/tmp/.X11-unix':'/tmp/.X11-unix' snakestrike-env SnakeStrike
```

[On to getting started with the GUI!](https://gwjensen.github.io/SnakeStrike/usage/config)

{% include links.html %}
