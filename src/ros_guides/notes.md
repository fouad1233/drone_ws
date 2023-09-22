###### Create catkin_ws:

```
mkdir catkin_ws
cd catkin_ws
mkdir src
catkin_make #inside catkin_ws
```

###### add source file to .bashrc:

```
source ~/catkin_ws/devel/setup.bash
```

###### Create a ROS Package:

```
catkin_create_pkg my_robot_conroller rospy turtlesim
~/catkin_ws$ catkin_make
```

###### Run listener and talker:

```
rosrun rospy_tutorials talker
rosrun rospy_tutorials listener
```

###### List current nodes:

```
rosnode list

rostopic list

$ rostopic info /chatter
Type: std_msgs/StringPublishers:
 * /talker_18118_1694715948305 (http://erhan-Acer:33685/)Subscribers:
 * /listener_18324_1694715984258 (http://erhan-Acer:45589/)
```

###### Get msg content

```
$ rosmsg show std_msgs/String
string data
```

###### Get topic content

```
rostopic echo /chatter
```

###### Rosservices:

```
rosservice list

$ rosservice info /turtle1/set_pen
Node: /turtlesim
URI: rosrpc://erhan-Acer:57297
Type: turtlesim/SetPen
Args: r g b width off

$ rossrv show turtlesim/SetPen
uint8 r
uint8 g
uint8 b
uint8 width
uint8 off

rosservice call /.....
```

###### Find ros package location:

```
rospack find gazebo_ros
```

###### Ros msg and srv types:

http://wiki.ros.org/msg      inthttp://wiki.ros.org/std_msgs

http://wiki.ros.org/std_srvs

###### Debug msg and srv

```
rosmsg list
rosmsg show ros_msgs/HardwareStatus
rossrv list 
rossrv show ros_msgs/ComputeDiskArea

```

###### Ros Paremeters

```
rosparam list
rosparam set /robot_name "my_robot"
rosparam set /sensors_read_freq 50
rosparam set /simulation_mode false
rosparam get /simulation_mode
```

###### Launch Files

```
roslaunch ros_bringup number_app.launch
```

###### Ros Bags

```
rosrun ros_guides hw_status_publisher.py 
rosbag record /my_robot/hw_status
rosbag info 2023-09-22-22-57-50.bag 

#stop the publisher node and listen the topic
rostopic echo /my_robot/hw_status #dont forget to listen the topic first
rosbag play 2023-09-22-22-57-50.bag
```
