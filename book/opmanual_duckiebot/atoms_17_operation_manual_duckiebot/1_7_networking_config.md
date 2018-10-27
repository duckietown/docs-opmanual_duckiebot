# Networking, a.k.a. the hardest part {#duckiebot-network status=ready}

This page is for the `DB18` configuration used in classes in 2018. For last year's instructions see [here](https://docs.duckietown.org/DT17/). 

<div class='requirements' markdown="1">

Requires: A Duckiebot that is initialized according to [](#setup-duckiebot).

Requires: Patience (channel your inner Yoda).

Result: A Duckiebot that you can connect to and that is connected to the internet.

</div>

The instructions here are ordered in terms of preference, the first being the most preferable and best. 


By default on boot your robot will look for a network with a "`duckietown`" SSID, unless you changed it in [the SD card flashing procedure](#burn-sd-card). You can connect to your robot wirelessly by connecting to that network. 

This page describes how to get your robot connected to the wide-area network (internet).

TODO: would be great if we had some network diagrams here.

## Testing if your Duckiebot is Connected to the Internet {#duckiebot-network-test}

Some networks block pings from passing through, so a better way is to execute:

    duckiebot $ sudo curl google.com


which will try to download the Google homepage. If it is successful, you should see an output like:


    <HTML><HEAD><meta http-equiv="content-type" content="text/html;charset=utf-8">
    <TITLE>301 Moved</TITLE></HEAD><BODY>
    <H1>301 Moved</H1>
    The document has moved
    <A HREF="http://www.google.com/">here</A>.
    </BODY></HTML>

## Option 1: Connect your Duckiebot to the internet through a Wifi router that you control

If you are working from your home, for example, you simply need to make the Duckiebot connect to your home network. You may have input the proper SSID and pwd when you initialized the SD card, in which case, your Duckiebot should be connected to the internet already. 

If you didn't enter the right SSID and password for your network or you want to change you need to connect to your robot somehow (e.g. with ethernet) and then edit the file `/etc/wpa_supplicant/wpa_supplicant.conf` as explained in the [Duckiebot initialization procedure](#burn-sd-card).

This is the best option. 

## Option 2: Bridge the internet connection through your laptop with ethernet

This method assumes that you can connect your laptop to a network but it is one that you don't control or is not open. For example, on campus many networks are more protected, e.g. with PEAP. In that case, it can be difficult to get your configurations right on the Duckiebot. An alternative is bridge the connection between your laptop and your Duckiebot whenever you need internet access on the robot. 

### Ubuntu

1. Connect your laptop to a wireless network.
2. Connect the Duckiebot to your laptop via an ethernet cable.

3. Make a new ethernet connection:

4. 1. Network Settings… (or run the command `nm-connection-editor`)
   2. Click "Add"
   3. Type -> Ethernet
   4. Connection Name: "Shared to Duckiebot"
   5. Select "IPV4" tab
   6. Select Method
   7. Select “Shared to other computers”
   8. Click apply.

Now, you should be able to SSH to your Duckiebot:

```
laptop $ ssh ![hostname]
```

Check whether you can access the internet from your Duckiebot:

```
duckiebot $ sudo curl google.com
```

Now, try to pull a Docker image:

```
duckiebot $ sudo docker pull duckietown/rpi-simple-server # This should complete successfully 
```

If the previous command does not work, you may need to change the system date. To do so, run the following command:

```
duckiebot $ sudo date -s "2018-09-18 15:00:00" # Where this is the current date in YYYY-MM-DD HH-mm-ss
```

### Mac

Untested instructions [here](https://medium.com/@tzhenghao/how-to-ssh-into-your-raspberry-pi-with-a-mac-and-ethernet-cable-636a197d055)


## Option 3: Push Docker Images from Laptop {#duckiebot-network-push status=beta}

Since we are primarily using the internet to pull Docker images, we can simply connect the laptop and the Duckiebot then push Docker images from the laptop over SSH like so:

```
laptop $ docker save duckietown/![image-name] | ssh -C ![hostname] docker load
```

Then the image will be available on your Duckiebot.

If you can connect to your laptop (e.g. through a router) but do not have internet access then you can proceed for now, but everytime you see a command starting with:

```
duckiebot $ docker run ...
```

note that you will need to pull onto your laptop and push to your Duckiebot in order to load the latest version of the image.
