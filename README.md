# Follow your SCITOS

Two Kuka youBots follow a SCITOS G5 to different parking spots 
using marker detection.

## Quick Overview 

This was a demo for the Long Night of Sciences 2014 in Magdeburg. 

There are three software parts in this repository. One is a MIRA-package, the 
second one is a ROS-package and the third one is an adapter that translates 
between MIRA and ROS.


## Demo Setup

One SCITOS G5 is running the MIRA-package and two Kuka youbots run the 
ROS-package. The task of the G5 is to get each Kuka from its current parking 
spot and bring it to another one.

Each Kuka is equipped with a Asus Xtion camera which can detect a marker that 
is applied to the G5. With this the youBot is able to follow the G5 to its next 
parking position.

The whole procedure works like this:

* The G5 queries all youBots until everyone has answered.
* The G5 drives to the first parking position and turns its marker towards it.
* The G5 tells the youBot in this parking spot to start following. 
* The youBot answers that it is now looking for the marker. 
* When the marker was found, the youBot tells the G5 to start driving.
* The G5 drives to the next parking position and tells the youBot to stop 
  following. 
* The G5 drives aside so it is not between the parking spot and the youBot 
  anymore.
* When the youBot got the message to stop following, it disables its marker 
  tracking and waits a few seconds for the G5 to move away.
* The youBot then drives straight on until it is in the parking spot. 
* The youBot turns 180 degrees so the camera faces towards the marker when the 
  G5 comes the next time.
* The youBot tells the G5 that it is finished. 
* The G5 drives to the next parking position and fetches the next youBot. 
* This is repeated until the demo is aborted or one of the robots runs out of 
  energy (probably the youBots).

## Software Setup

### Software Requirements

For this demonstration to work [MIRA](http://www.mira-project.org/) and 
[ROS](http://www.ros.org/) have to be installed on the G5.
This demonstration used version 0.10.0-20140415 of the MIRA packages and ROS Hydro.

For this demo the base installation of ROS is sufficient:  

    sudo apt-get install ros-hydro-ros-base

Now get the source of this project:

    cd ~
    git clone https://github.com/dirksteindorf/followYourScitos.git

If this needs to be in a different directory, all the following pathes have to 
be adjusted, as well as the last line in `~/followYourScitos/MIRA/followYourScitos/PackageList.txt`

### MIRA Setup

The following lines have to be added to the `~/.bashrc`:

    export MIRA_PATH=$MIRA_PATH:/localhome/demo/followYourScitos
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/localhome/demo/followYourScitos/lib

The file now needs to be sourced:

    source ~/.bashrc

Now the MIRA unit can be built:

    cd ~/followYourScitos/MIRA/followYourScitos
    make

### ROS

The ROS part of this setup is in a different repository and won't be uploaded 
here. As a replacement there are some helper scripts which will function as 
surrogates for the youBots. 
And because nothing worked for a while there's also a script that surrogates the 
G5.


#### ROS Setup

To run these scripts Python and ROS Hydro have to be installed. Since the youBots 
and the G5 use Ubuntu, Python is already installed and only ROS is missing. 

In order to compile the Adapter the following lines have to be added to the `~/.bashrc`

    source /opt/ros/hydro/setup.bash
    export ROS_PACKAGE_PATH=/localhome/demo/followYourScitos/Adapter:$ROS_PACKAGE_PATH  

Both the ROS scripts and the Adapter need to know where the ROS master is running. 
For example, add this to `~/.bashrc` and replace the `127.0.0.1` by the IP of the 
computer the roscore is running on:  

    export ROS_MASTER_URI=http://127.0.0.1:11311

The file now needs to be sourced:

    source ~/.bashrc

### Adapter

All prerequisites should be met, so the adapter can be built:
    
    cd ~/followYourScitos/Adapter
    make

## Running the software

Run the ROS master in one terminal:  

    roscore

MIRA Center is started with a configuration file. There's only one in this repository that looks like this:

    <root>
        <include file="${findpkg SCITOSConfigs}/etc/SCITOS-Pilot.xml"/>
    
        <!--<unit id="doubleKukaParking" class="kukaParking::doubleKukaParking" />-->
        <!--<unit id="singleKukaParking" class="kukaParking::singleKukaParking" />-->
        <unit id="kukaController" class="kukaParking::kukaRemoteController" />
    </root>

The two commented lines are for driving autonomously with one or two youBots. The third line is currently active. there you control the G5 using a Gamepad. In this scenario only one youBot follows the G5. Using the buttons X and Y you tell the youBot to start/stop following.  
Switching between these modes is accomplished through commenting out the stuff that's not needed and uncommenting the needed line.

Now start MIRA Center with the chosen configuration:

    miracenter -c ~/followYourScitos/MIRA/followYourScitos/domains/kukaParking/etc/navConfig.xml

Now the adapter can be started: 

    ~/followYourScitos/Adapter/bin/miraadapter -k 127.0.0.1:1234

This, of couse, is only the case if the adapter is running on the same machine as MIRA Center. Otherwise the IP has to be adjusted accordingly.

Now the youBot scripts can be started. For two youBots start the two scripts:

    python ~/followYourScitos/ROS/helper_scripts/pseudoKuka1.py
    python ~/followYourScitos/ROS/helper_scripts/pseudoKuka2.py

For one youBot (autonomous or not) start only one:

    python ~/followYourScitos/ROS/helper_scripts/pseudoKuka1.py
    
The following table shows the function of each button on the Gamepad (Logitech F710).

<table>
    <tr>
        <td>Button</td>
        <td>Function</td>
    </tr>

    <tr>
        <td>A</td>
        <td>set velocity to 0</td>
    </tr>

    <tr>
        <td>B</td>
        <td>set velocity to 0</td>
    </tr>

    <tr>
        <td>X</td>
        <td>tell the youBot to start following</td>
    </tr>

    <tr>
        <td>Y</td>
        <td>tell the youBot to stop following</td>
    </tr>

    <tr>
        <td>LB</td>
        <td>twinkle left</td>
    </tr>

    <tr>
        <td>RB</td>
        <td>twinkle right</td>
    </tr>

    <tr>
        <td>LT</td>
        <td>minimal speed</td>
    </tr>

    <tr>
        <td>RT</td>
        <td>maximal speed (can be configured in MIRA Center)</td>
    </tr>

    <tr>
        <td>Start</td>
        <td>enable/disable driving mode</td>
    </tr>

    <tr>
        <td>Back</td>
        <td>emergency stop (active braking) and lock driving mode,<br />
        unlock with Start</td>
    </tr>

    <tr>
        <td>Mode</td>
        <td>switches D-Pad and left stick<br />
            don't press it, it's really annoying</td>
    </tr>

    <tr>
        <td>Vibration</td>
        <td>has no effect on the robot, he just doesn't care</td>
    </tr>

    <tr>
        <td>D-Pad up</td>
        <td>increase velocity by 0.1 m/s</td>
    </tr>

    <tr>
        <td>D-Pad down</td>
        <td>decrease velocity by 0.1 m/s,<br />
            drive backwards</td>
    </tr>

    <tr>
        <td>D-Pad left</td>
        <td>increase counter clockwise rotation by 0.1 rad/s</td>
    </tr>

    <tr>
        <td>D-Pad right</td>
        <td>increase clockwise rotation by 0.1 rad/s</td>
    </tr>

    <tr>
        <td>Left stick</td>
        <td>orientation of the head (90° steps)</td>
    </tr>
</table>
