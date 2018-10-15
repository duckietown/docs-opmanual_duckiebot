# Docker basics {#preliminaries-docker-basics status=draft}

Assigned: Aleks

## What is Docker? {#basic-docker-what-is}

Docker is used to perform operating-system-level virtualization, something often referred to as containerization. While Docker is not the only software that does this, it is by far the most popular one.

Containerization is a process that allows partitioning the hardware and the kernel of an operating systems in such a way that different *containers* can co-exist on the same system independently from one-another. Programs running in such a container have access only to the resources they are allow to and are completely independent of libraries and configurations of the other containers and the host machine. Because of this feature Docker containers are extremely portable.

Containers are often compared to virtual machines (VMs). The main difference is that VMs require a host operating system (OS) with a hypervisor and a number of guest OS, each with their own libraries and application code. This can result in a significant overhead. Imagine running a simple Ubuntu server in a VM on Ubuntu: you will have most of the kernel libraries and binaries twice and a lot of the processes will be duplicated on the host and on the guest. Containerization, on the other hand, leverages the existing kernel and OS and adds only the additional binaries, libraries and code necessary to run a given application. See the illustration bellow.

<figure class="flow-subfigures">  
    <figcaption>Comparison between containers and VMs (from [docker.com](https://docs.docker.com/get-started/))</figcaption>
    <figure>
        <figcaption>Using containers</figcaption>
        <img style='width:20em' src="images/docker-containerVM.png"/>
    </figure>
    <figure>  
        <figcaption>Using VMs</figcaption>
        <img style='width:20em' src="images/docker-containerVM2.png"/>
    </figure>
</figure>  

Because containers don't need a separate OS to run they are much more lightweight than VMs. This makes them perfect to use in cases where one needs to deploy a lot of independent services on the same hardware or to deploy on not-especially powerful platforms, such as Raspberry Pi - the platform Duckiebots use.

Containers allow for reuse of resources and code, but are also very easy to work with in the context of version control. If one uses a VM, they would need to get into the VM and update all the code they are using there. With a Docker container, the same process is as easy as pulling the container image again.

## How does Docker work? {#basic-docker-how-works}
You can think that Docker containers are build from Docker images which in turn are build up of Docker layers. So what are these?

Docker images are build-time constructs while Docker containers are run-time constructs. That means that a Docker image is static, like a `.zip` or `.iso` file. A container is like a running VM instance: it starts from a static image but as you use it, files and configurations might change.

Docker images are build up from layers. The initial layer is the *base layer*, typically an official stripped-down version of an OS. For example, a lot of the Docker images we run on the Duckiebots have `rpi-ros-kinetic-base` as a base.

Each layer on top of the base layer constitutes a change to the layers bellow. The Docker internal mechanisms translate this sequence of changes to a file system that the container can then use. If one makes a small change to a file, then typically only a single layer will be changed and when Docker attempts to pull the new version, it will need to download and store only the changed layer, saving space, time and bandwidth.

In the Docker world images get organized by their repository name, image name and tags. As with Git and GitHub, Docker images are stored in image registers. The most popular Docker register is called DockerHub and it is what we use in Duckietown. Let's look at one image stored on DockerHub:

`duckietown/rpi-ros-kinetic-base:master18`

Here the repository name is `duckietown` the image name is `rpi-ros-kinetic-base` and the tag is `master18`. All Duckietown-related images are in this repository. The images themselves can be very different and for various applications.

However, sometimes a certain image might need to have several different versions. These can be designated with *tags*. For example, the `master18` tag means that this is the image to be used with the 2018 version of the hardware. It is not necessary to specify a tag. If you don't, Docker assumes you are interested in the image with `latest` tag, should such an image exist.

## Frequently-used commands {#basic-docker-commands}
Let's go through some of the most frequently used operations in Docker:

### Working with images
If you want to get a new image from a Docker register (e.g. DockerHub) on your local machine then you have to *pull* it. For example, you can get an Ubuntu 18.04 image by running the following command:

    laptop $ docker pull library/ubuntu:18.04

You will now be able to see the new image you pulled if you run:

    laptop $ docker image list

If you don't need it, or if you're running down on storage space, you can remove an image by simply running:

    laptop $ docker image rm ubuntu:18.04

You can also remove images by their `IMAGE ID` as printed by the `list` command. Sometimes you might have a lot of images you are not using. You can easily remove them all with:

    laptop $ docker image prune

However, be careful not to delete something you might actually need. Keep in mind that you can't remove images that a container is using. To do that, you will have to stop the container, remove it, and then you can remove the related images.

If you want to look into the heart and soul of your images, you can use the commands `docker image history` and `docker image inspect` to get a detailed view.

TODO: Add information on how to change the storage place for images

### Working with containers

Containers are the run-time equivalent of images. When you want to start a container, Docker picks up the image you specify, creates a file system from its layers, attaches all devices and directories you want, "boots" it up, sets up the environment up and starts a pre-determined process in this container. All that magic happens with you running a single command: `docker run`. You don't even need to have pulled the image beforehand, if Docker can't find it locally, it will look for it on DockerHub. Here's a simple example:

    laptop $ docker run ubuntu

This will take the `ubuntu` image with `latest` tag and will start a container from it. This won't do much, in fact the container will immediately exit as it has nothing to execute. When the main process of a container exits, the container exits as well. By default this `ubuntu` image runs `bash` and as you don't pass any commands to it, it exits immediately. This is no fun, though. Let's try to keep this container alive for some time:

    laptop $ docker run -it ubuntu

Now you should see something like:

    docker $ root@73335ebd3355:/#

Keep in mind that the part after `@` will be different, that is your container ID. You are now in your new `ubuntu` container! Try to play around, you can try to use some basic `bash` commands like `ls`, `cd`, `cat` to make sure that you are not in your host machine.

You can check which containers you are running right now. Open a new terminal window (don't close the other one yet) and type:

    laptop $ docker ps

or

    laptop $ docker container list

These commands list all running containers. Now you can go back to your `ubuntu` container and type `exit`. This will bring you back to you host shell and will stop the container. If you again run the `docker ps` command you will see nothing running. So does this mean that this container and all changes you might have made in it are gone? Not at all, `docker ps` and `docker container list` only list the *currently running* containers. You can see all containers, including the stopped ones with:

    laptop $ docker container list -a

here `-a` stands for *all*. You will see you have two `ubuntu` containers here. There are two containers because every time you use `docker run`, a new container is created. Note that their names seem strangely random, we could have added custom, more descriptive names, but more on this later.

We don't really need these containers so let's get rid of them:

    laptop $ docker container rm ![container_name] ![container_name]

You need to put your container names after `rm`. Using the containr IDs instead is also possible. Note that if the container you are trying to remove is still running you will have to first stop it.

You might need to do some other operations with containers. For example, sometimes you want to start or stop an existing container. You can simply do that with:

    laptop $ docker container start ![container_name]
    laptop $ docker container stop ![container_name]
    laptop $ docker container restart ![container_name]

Imagine you are running a container in the background. The main process is running but you have no shell attached. How can you interact with the container? You can open a terminal in the container with:

    laptop $ docker attach ![container_name]

Often we will ask you to run containers with more sophisticated options than what we saw before. Look at the following example: (don't try to run this, it will not do much).

    laptop $ docker -H hostname.local run -dit --privileged --name joystick --network=host -v /data:/data duckietown/rpi-duckiebot-joystick-demo:master18

Here's an explanation of some of the options we use most often in Duckietown:

<col4 figure-id="tab:docker-run-tab" class="labels-row1">
    <figcaption><code>docker run</code> options</figcaption>
    <span>Short command </span>
    <span>Full comamnd</span>
    <span>Example</span>
    <span>Explanation</span>
    <span>`-i`</span>
    <span>`--interactive`</span>
    <span></span>
    <span>Keep STDIN open even if not attached, typically used together with `-t`.</span>
    <span>`-t`</span>
    <span>`--tty`</span>
    <span></span>
    <span>Allocate a pseudo-TTY, gives you terminal access to the container, typically used together with `-i`.</span>
    <span>`-d`</span>
    <span>`--detach`</span>
    <span></span>
    <span>Run container in background and print container ID.</span>
    <span></span>
    <span>`--name`</span>
    <span>`--name joystick`</span>
    <span>Sets a name for the container. If you don't specify one, a random name will be generated.</span>
    <span>`-v`</span>
    <span>`--volume`</span>
    <span>`-v /home/myuser/data:/data`</span>
    <span>Bind mount a volume, exposes a folder on your host (`/home/myuser/data`) as a folder in your container (`/data`). Be very careful when using this.</span>
    <span>`-p`</span>
    <span>`--publish`</span>
    <span>`-p 8082:8080`</span>
    <span>Publish a container's port(s) to the host, necessary when you need a port to communicate with a program in your container. Your host's port `8082` will be mapped to the container's `8080`.</span>
    <span>`-d`</span>
    <span>`--device`</span>
    <span>`-d /dev/mmcblk0`</span>
    <span>Similar to `-v` but for devices. This grants the container access to the `/dev/mmcblk0` device. Be very careful when using this.</span>
    <span></span>
    <span>`--privileged`</span>
    <span></span>
    <span>Give extended privileges to this container. That includes access to **all** devices. Be **extremely** careful when using this.</span>
    <span></span>
    <span>`--rm`</span>
    <span></span>
    <span>Automatically remove the container when it exits.</span>
    <span>`-H`</span>
    <span>`--hostname`</span>
    <span>`docker -H duckiebot.local`</span>
    <span>Specifies remote host name, for example when you want to execute the command on your Duckiebot, not on your computer.</span>
    <span></span>
    <span>`--help`</span>
    <span></span>
    <span>Prints information about these and other options.</span>
</col4>

### Some other useful commands

Sometimes your docker system will be clogged with images, containers and what not. You can use `docker system prune` to clean it up. Keep in mind that this command will delete **all** containers that are not currently running and **all** images not used by running containers. So be extremely careful when using it.

Often, for simple operations and basic commands, one can use Portainer. Portainer is itself a Docker container that allows you to control the Docker daemon through your web browser. You can install it by running:

    laptop $ docker volume create portainer_data
    laptop $ docker run -d -p 9000:9000 --name portainer --restart always -v /var/run/docker.sock:/var/run/docker.sock -v portainer_data:/data portainer/portainer

Note that Portainer comes pre-installed on your Duckiebot so you don't need to run the above command to access the images and containers on your robot. You still might want to set it up for your laptop.

## Further resources

There is much more that you can learn to do with Docker. Here are some resources you can look up:

- [Duckietown Containerization manual](#part:docker-devel)
- Docker official Get Started tutorial: [https://docs.docker.com/get-started/](https://docs.docker.com/get-started/)
- Docker Curriculum: [https://docker-curriculum.com/](https://docker-curriculum.com/)
- Docker Deep Dive, by Nigel Poulton
