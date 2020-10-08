# YouBot

ROS packages for KUKA YouBot Robot

**Maintainer:** [Miguel Cuartin Ordaz](https://www.linkedin.com/in/macuartin/)
  
## Installation

### Basic Requirements

  1. Install [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) (**Base Install** Recommended)

### Repository Installation

Go to your ROS working directory. e.g.
```bash
cd ~/catkin_ws/src
``` 
Clone the required repositories:
```bash
git clone https://github.com/fsuarez6/lenny.git
```
Install any missing dependencies using rosdep:
```bash
rosdep update
rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO
``` 
Now compile your ROS workspace. e.g.
```bash
cd ~/catkin_ws && catkin_make
``` 

### Testing Installation

Be sure to always source the appropriate ROS setup file:
```bash
source ~/catkin_ws/devel/setup.bash
``` 
You might want to add that line to your `~/.bashrc`
