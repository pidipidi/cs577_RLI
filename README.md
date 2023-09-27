# CS577: Robot Learning and Interaction

# Installation
## Pre-requites for this tutorial
Please, install Ubuntu 20.04 and [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) with dependencies:
~~~~bash
sudo apt-get install ros-noetic-qt-gui ros-noetic-driver-common ros-noetic-gazebo-ros-control ros-noetic-gazebo-ros-pkgs ros-noetic-ros-control ros-noetic-control-toolbox ros-noetic-realtime-tools ros-noetic-ros-controllers ros-noetic-xacro python3-wstool -y
sudo pip3 install beizer bezier
~~~~

Before install the Baxter simulator you need to install libqt4-dev:
~~~~bash
sudo add-apt-repository ppa:rock-core/qt4
sudo apt update
sudo apt install libqt4-dev
~~~~

## Installation of your project repository
~~~~bash
source /opt/ros/noetic/setup.sh
~~~~

Move to any directory you want to place the class code. Then, you can create a workspace,
~~~~bash
mkdir -p ./catkin_ws/src
cd ./catkin_ws/src/
catkin_init_workspace
~~~~

Let's clone the the class repo!
~~~~bash
git clone git@github.com:pidipidi/cs577_RLI.git
~~~~

To simulate a Baxter robot using a Gazebo simulator, you have to install the simulation-related packages,
~~~~bash
cd cs577_RLI
wstool init .
wstool merge third_party_repos.rosinstall
wstool update
~~~~

Then, install dependencies and build it!
~~~~bash
cd ../..
rosdep install --from-paths src --ignore-src --rosdistro=noetic -y
catkin_make
~~~~
If you want catkin_build, you can use it instead of catkin_make.


Lastly, load the new environmental variables and also add to path the environmental setup to your bashrc
~~~~bash
cp ./src/cs577_RLI/baxter.sh ./
./baxter.sh sim
~~~~

You can verify your ROS install by confirming the environmental variables
~~~~bash
env| grep ROS
~~~~

Make sure that if ROS_MASTER_URI and ROS_ROOT, ETC are properly setup. 


# Assignment Links 
- [Assignment I](assignment_1/README.md)
- [Assignment II](assignment_2/README.md)
- [Assignment III]()

# ETC
## Command-line tools
There are many useful command-line tools like rostopic, rqt_graph, rosbag, etc. Please, see http://wiki.ros.org/ROS/CommandLineTools



## Tutorials for Baxter
The instructions for the the Baxter can be found in here:
http://sdk.rethinkrobotics.com/wiki/Baxter_Simulator. Please, note that the instruction works for ROS Melodic only. 


