# Minor project Geen moer aan.

### ROS Installation

For more information & troubleshooting: [ROS Indigo installation](http://wiki.ros.org/indigo/Installation/Ubuntu)

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
```

```
sudo apt-get install ros-indigo-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Setting up workspace

```
cd ~
mkdir -p workspace/src
cd workspace/src
catkin_init_workspace
cd ..
catkin_make
echo "source /home/[username]/workspace/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Installing project dependencies

``` 
sudo apt-get install ros-indigo-moveit-full ros-indigo-abb-driver ros-indigo-controller-manager ros-indigo-opencv3 ros-indigo-industrial-robot-simulator
```

### Installing project

```
cd src
git clone https://github.com/RTwTools/GeenMoerAan.git
cd ..
catkin_make
```

### Setting up the User Interface:
```
cd Controller-executable
chmod +x *
```
### Running the User Interface:
```
./Controller
```
Or  double click on the binary executable in "Controller-executable"

Enter your workspace directory on the GUI,

Notice:  this directory should be unique on the HDD !!!

