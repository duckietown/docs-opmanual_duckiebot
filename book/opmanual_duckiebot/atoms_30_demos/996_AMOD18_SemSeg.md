# AMOD18 SemSeg {#demo-semseg status=draft}

<div class='requirements' markdown="1">

Requires: A Duckiebot able to take logs from the camera (a working ros-picam container)

Requires: A calibrated camera

Requires: A duckietown complying with the usual standards

Requires: A laptop with python2 and ROS installed

Requires: Bash shell command line

</div>


## Video of expected results {#demo-semseg-expected}

First, we show a video of the expected behavior (if the demo is succesful).

## Duckietown setup notes {#demo-semseg-duckietown-setup}

A duckietown complying with the usual standards is required. In particular, the segmentation algorithm is designed to work under the following conditions:

* Uninterrupted white lines complying with standards
* Yellow (middle) lines complying with standards
* Only ducks or duckiebots are allowed to be on the road, cones or other objects should be removed as they won't be detected as obstacles
* Humans, other animals or objects are only allowed outside of the driving lanes
* No particular lighting conditions are required


## Duckiebot setup notes {#demo-semseg-duckiebot-setup}

No special requirements for duckiebots are needed, except the ability of recording logs from the camera.
Maybe, something regarding the movidus stick if we plan to use the software on the bot

## Pre-flight checklist {#demo-semseg-pre-flight}

Check: you have python2 and pip installed

Check: you have ROS installed. Run >>roscore in terminal too check this.

## Demo instructions {#demo-semseg-run}
Open a new terminal

All these commands have to be run in the bash command line.

If you are using zsh, enter the command >>bash every time you open a new terminal          


    laptop $ git clone https://github.com/zj-dong/duckietown-semantic-segmentation.git  
    laptop $ cd     
    laptop $ mkdir -p ~/catkin_ws3/src    
    laptop $ cd ~/catkin_ws3    
    laptop $ catkin_make    
    laptop $ cd duckietown-semantic-segmentation    
    laptop $ cp -r src/duckietown_msgs/ src/tf_sem_seg/ ~/catkin_ws3/src    
    laptop $ chmod +x requirements.txt    
    laptop $ cd ~/catkin_ws3    
    laptop $ catkin_make    
    laptop $ sudo pip2 install -r requirements.txt    
    laptop $ gedit node_launch.sh        


In these two lines you need to replace """your duckiebot""" with the name of your duckiebot:

export DUCKIEBOT_NAME="""your duckiebot"""  
export ROS_MASTER_URI=http://"""your duckiebot""".local:11311  

Save and close the file


Now, check if you can ping your bot. If you can't, you have problems with the connection. Please refer to the duckiebot manual for this.

If you can ping your bot, go on portainer (http://.![robot name]:9000/#/containers) and make sure that ros-picam and roscore containers are running.

Enter the following commands      

    laptop $ cd ~/catkin_ws3    
    laptop $ source devel/setup.bash    
    laptop $ sh node_launch.sh    

Well done". You are now receiving images from the Duckiebot and processing them on your laptop. Wait until you can read "Predicting" and "Finish prediction", then let the node running without stopping it.

Now, in order to visualize your the processed images, run the following commands in the terminal (make sure you are in bash)

Open a new terminal          

    laptop $ source ~/catkin_ws3/devel/setup.bash    
    laptop $ export ROS_MASTER_URI=http://![robot name].local:11311    
    laptop $ rostopic list        

Check whether there is a /"""your duckiebot"""/prediction_images. If you can find it, proceed as follows.        

    laptop $ rosrun rviz rviz          

Wait for rviz to load. Once the interface is ready, click on "Add" (bottom right) and choose "by topic -> /prediction_images/Image"  
You can make the image larger by dragging the boundaries of the window.

The first image is the image obtained from the Duckiebot. The second image shows the results of the segmentation algorithm. The third image shows both images together.

Please consider that the algorithm was not trained on duckietown images but on a standard semantic segmentation database with images from "real world" roads. This happened because we were unable to obtain labelled images for duckietown.
Since the network was pre-trained, it was impossible for us to reduce its size and make it lightweight. In particular, the images from the Duckiebot, are resized to 1024 x 2048 images in order to be processed by the network and this is obviously inefficient.


## Troubleshooting {#demo-semseg-troubleshooting}

Add here any troubleshooting / tips and tricks required.

## Demo failure demonstration {#demo-semseg-failure}

Finally, put here a video of how the demo can fail, when the assumptions are not respected.
