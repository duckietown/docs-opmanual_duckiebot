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

    duckiebot $ sudo wget -O- google.com


which will try to download the Google homepage. If it is successful, you should see an output like:


    --2018-09-20 14:55:31--  http://google.com/
    Resolving google.com... 172.217.13.174
    Connecting to google.com|172.217.13.174|:80... connected.
    HTTP request sent, awaiting response... 301 Moved Permanently
    Location: http://www.google.com/ [following]
    --2018-09-20 14:55:31--  http://www.google.com/
    Resolving www.google.com... 172.217.13.164
    Connecting to www.google.com|172.217.13.164|:80... connected.
    HTTP request sent, awaiting response... 200 OK
    Length: unspecified [text/html]
    Saving to: ‘STDOUT’
    ...


## Option 1: Connect your Duckiebot to the internet through a Wifi router that you control

If you are working from your home, for example, you simply need to make the Duckiebot connect to your home network. You may have input the proper SSID and pwd when you initialized the SD card, in which case, your Duckiebot should be connected to the internet already. 

If you didn't enter the right SSID and password for your network or you want to change you need to connect to your robot somehow (e.g. with ethernet) and then edit the file `/etc/wpa_supplicant/wpa_supplicant.conf`.

This is the best option. 

## Option 2: Bridge the internet connection through your laptop with ethernet

This method assumes that you can connect your laptop to a network but it is one that you don't control or is not open. For example, on campus many networks are more protected, e.g. with PEAP. In that case, it can be difficult to get your configurations right on the Duckiebot. An alternative is bridge the connection between your laptop and your Duckiebot whenever you need internet access on the robot. 

### Ubuntu

1. Connect your laptop to the network

2. Connect the Duckiebot to your laptop via an ethernet cable.

3. Make a new ethernet connection:

4. 1. Network Settings… (or run the command nm-connection-editor)
   2. Click "Add"
   3. Type -> Ethernet
   4. Connection Name: "Shared to Duckiebot"
   5. Select "IPV4" tab
   6. Select Method
   7. Select “Shared to other computers”
   8. Click apply.



### Mac

Untested instructions [here](https://medium.com/@tzhenghao/how-to-ssh-into-your-raspberry-pi-with-a-mac-and-ethernet-cable-636a197d055)



## Option 3: Push Docker Images from Laptop {#duckiebot-network-push status=beta}

TODO: Needs Testing

Since we are primarily using the internet to pull Docker images, we can simply connect the laptop and the Duckiebot and push docker images down from the laptop. 

```
laptop $ docker save duckietown/![image-name] | ssh -C ![username]@![hostname].local docker load
```

Then the image will be available on your Duckiebot.

If you can connect to your laptop (.e.g through a router) but you don't have internet access then forge ahead for now but everytime you see a:

```
duckiebot $ docker run ...
```

Note that you will have to do this procedure or pulling onto your laptop and send to your Duckiebot in order to get the latest version of the image. 
