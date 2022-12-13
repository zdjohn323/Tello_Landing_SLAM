#Tello_Landing_SLAM 
##ROS Install
Setup your computer to accept software from packages.ros.org. : 
```sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' ```
Set up your keys 
```sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key  C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 ```
Installation 
First, make sure your Debian package index is up-to-date: 
```sudo apt update ```
Desktop-Full Install: (Recommended) : ROS, rqt, rviz, robot-generic libraries,  2D/3D simulators and 2D/3D perception 
```sudo apt install ros-melodic-desktop-full ```
Initialize rosdep 
Before you can use ROS, you will need to initialize rosdep. rosdep enables you to easily install system  dependencies for source you want to compile and is required to run some core components in ROS. 
```sudo rosdep init ```
```rosdep update ```
##Environment setup 
It's convenient if the ROS environment variables are automatically added to your bash session every  time a new shell is launched: 
```echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc ```
```source ~/.bashrc```
Dependencies for building packages 
Up to now you have installed what you need to run the core ROS packages. To create and manage your  own ROS workspaces, there are various tools and requirements that are distributed separately. For  example, rosinstall is a frequently used command-line tool that enables you to easily download many  source trees for ROS packages with one command. To install this tool and other dependencies for  building ROS packages, run: 
```sudo apt install python-rosinstall python-rosinstall-generator python-wstool build essential ```
##Install Prerequisites 
catking tools 
First you must have the ROS repositories which contain the .deb for catkin_tools: 
```sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main"  > /etc/apt/sources.list.d/ros-latest.list' ```
```wget http://packages.ros.org/ros.key -O - | sudo apt-key add - ```
Once you have added that repository, run these commands to install catkin_tools: 
```sudo apt-get update ```
```sudo apt-get install python-catkin-tools ```
Eigen3 
Required by g2o. Download and install instructions can be found here. Otherwise Eigen can be  installed as a binary with: 
```sudo apt install libeigen3-dev ```
ffmpeg 
```sudo apt install ffmpeg ```
Python catkin tools (probably already installed) 
```sudo apt-get install python-catkin-tools ```
Joystick drivers 
Tested it only on melodic. 
```sudo apt install ros-melodic-joystick-drivers```
Python PIL 
```sudo apt-get install python-imaging-tk ```

##Github based Prerequisites 
Pangolin (used in orbslam2) 
Based on https://github.com/stevenlovegrove/Pangolin 
```cd ~/(favorite directory)/ ```
```git clone https://github.com/stevenlovegrove/Pangolin.git ```
```sudo apt install libgl1-mesa-dev ```
```sudo apt install libglew-dev ```
```sudo apt-get install libxkbcommon-dev ```
```cd Pangolin ```
```mkdir build ```
```cd build ```
```cmake .. ```
```cmake --build ```
h264decoder 
  Baed on https://github.com/DaWelter/h264decoder 
```cd ~/(favorite directory/``` 
```git clone https://github.com/DaWelter/h264decoder.git ```
Download the version V1 which is the old version, the old version geneartes  libh264decoder.so 
``` git checkout tags/V1```
Inside h264decoder.cpp replace PIX_FMT_RGB24 with AV_PIX_FMT_RGB24 
```mkdir build ```
```cd build ```
```cmake .. ```
```make``` 
now copy it to python path 
```sudo cp ~/ROS/h264decoder/libh264decoder.so /usr/local/lib/python2.7/dist-packages ```
-lEIGEN: 
Update Eigen version from 3.2 to 3.3.9 Download Eigen 3.3.9 
   ```git clone && make && make install ```
Then replace all the find_package(Eigen3 REQUIRED) to 
list(APPEND CMAKE_INCLUDE_PATH "/usr/local/include")  
find_package (Eigen3 3.3 REQUIRED NO_MODULE) 
```sudo apt-get install ros-kinetic-cv-bridge```
##Tello_Landing_SLAM 
Cloning repo from github 
In user’s favioute directory: 
```git clone https://github.com/zdjohn323/Tello_Landing_SLAM.git```

Unzip the Vocabulary file in /Tello_ROS_ORBSLAM/ROS/tello_catkin_ws/src/ORB_SLAM2_2d
Installing TelloPy 
based on https://github.com/dji-sdk/Tello-Python and https://github.com/hanyazou/TelloPy 
```cd ~/(name of directory)/Tello_Landing_SLAM/TelloPy ```
```sudo python setup.py install ```
Installing dependencies for ROS 
```cd ~/(name of directory)/Tello_Landing_SLAM/ROS/tello_catkin_ws/```
```sudo rosdep init ```
```rosdep update ```
```rosdep install --from-paths src --ignore-src -r -y ```

Build the code: 
```cd ~/(your favorite directory)//Tello_Landing_SLAM/ROS/tello_catkin_ws/ ```
```catkin init ```
```catkin clean ```
```catkin build ```
```Source devel/setup.bash```
If it doesn’t work, make sure you changed the makefile to the wanted version of ROS 
Add the enviroment setup to bashrc 
```echo "source $PWD/devel/setup.bash" >> ~/.bashrc ```
```source ~/.bashrc ```

After successfully build the code:
To run the code:
```cd ~/(your favorite directory)//Tello_Landing_SLAM/ROS/tello_catkin_ws/```
Tello Controller with ORB_SLAM2
```roslaunch flock_driver orbslam2_with_cloud_map.launch```
```	roscore```
```rosrun ORB_SLAM2_2d Monopub src/ORB_SLAM2_2d/Vocabulary/ORBvoc.txt src/ORB_SLAM2_2d/Monocular/Tello.yaml -1 /tello/camera/image_raw```
```rosrun ORB_SLAM2 Monosub 5 3 29 -25 48 -12 0.55 0.50 1 5``` (change the numbers according to 2d-grid-mapping.pdf)
