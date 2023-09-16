# Installation

Clone the repo:

```
git clone https://github.com/gtu-ros/drone_ws.git
```

or

```
git clone https://<oauth-key-goes-here>@github.com/gtu-ros/drone_ws.git
```

Initialize the drone workspace:

```
cd ~/drone_ws
catkin init
```

Build workspace

```
cd ~/drone_ws
catkin build
```

Add setup.bash to your .bashrc

```
echo "source ~/drone_ws/devel/setup.bash" >> ~/.bashrc
```

Update global variables

```
source ~/.bashrc
```

You can read ROS_Guides folder ROS Noetic guides.

This repo was clonned from https://github.com/erhangk/ROS_Guides.git

### Installing gazebo models

Gazebosim link:

`http://models.gazebosim.org/`
