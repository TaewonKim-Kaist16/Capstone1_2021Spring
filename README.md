# Capstone1 2021 Spring

KAIST 2021 Spring Capstone Design 1

This is a repository for capstone design 1 codes. All the codes are just for your reference, so change the code as necessary (e.g. topic name, ball size and color).

To download the package, write the command below in ~/(catkin work space name)/src terminal.
```console
git clone https://github.com/kaistcapstone/Capstone1_2021Spring.git
```


## ball_detection

This node needs opencv if you have not installed opencv follow below link.

https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html

Node for detecting red and green balls.

It publishes relative red ball positions as a “/red_position” topic and relative green ball position as a “/green_position” topic.
The topic type is core_msgs::ball_position.

In the core_msgs::ball_position, the direction of img_x and img_y is shown below. You can just say img_x as right direction and img_y as front direction.

vision sensor view direction: ^  
img_x direction: >  
img_y direction: ^  


### Usage

```console
rosrun ball_detection ball_detection_node
```

### Error
If large error in relative position or any bug occurs, record the image topic from CoppeliaSim using 'rosbag' command.

## line_distance

This node needs opencv if you have not installed opencv follow below link.

https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html

Node for detecting planes with line and calculating relative distance and slope angle for them.

It publishes relative distances and slope angle of the planes as '/plane_info' topic.

Unit of slope is degree, and unit of distance is meter.

Slope and distance informations are sorted from nearest line to farthest line.

Now, it works with top camera(front view).

### plane_info.msg
This message has  
Header header  
int32 size  
float32[] slope  
float32[] distance  
size is size for slope and distance vectors.
slope and distance are relative to the depth camera.

### Usage

```console
rosrun line_distance line_distance_node
```

### Error
Please, check you made below line as a comment('--') in the lua script of kinect.
depth_d['is_bigendian'] = 1  
->  
--depth_d['is_bigendian'] = 1  

## coppeliasim_models

map_ver_x.ttt, my_robot_ver_x.ttm files

Map and my_robot can be updated later, so please check the notice board on klms regularly.

Just drag and drop files to coppeliasim window.

For reference, urdf file for my_robot is uploaded.

1. mylidar_hokuyo.ttm

- ROS

frame : base_scan

topic : /scan

msg type : sensor_msgs::LaserScan

- Lidar specs

rate : 5Hz

resolution : 1 deg -> 360 points per 1 scan

min range : 0.12 m

max range : 3.5 m


2. myimu.ttm

- ROS

frame : imu

topic : /imu

msg type : sensor_msgs::Imu


3. mykinect.ttm

- Camera parameters

FOV (field of view) : 57 deg

W : 640 pixels

H : 480 pixels

f : 589.37 pixels

## ball_approach

This node needs opencv if you have not installed opencv follow below link.

https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html

This node is activated if the topic "/is_align" which has type std_msgs::Bool is true.

When this node is activated, it assumes that the car is aligned with a target ball, and the distance to the ball is close enough.

If this node is activated, it makes the gripper main position be -0.055.

Also, it makes the gripper arm position be 0.

Next, it makes the car go straight until below camera detects number of red points larger than some threshold (red_cnt_threshold).

If it successfully approached the ball, it publishes "/is_align" with false to make this node disable, and also publishes "/ball_approach_success" with true.

Also, if it successfully approached, it tights gripper arm to 0.035.

If it cannot approach ball for 4 sec in simulTime, it stops approaching and publishes "/is_align" with false to make this node disable, and also publishes "/ball_approach_success" with false.

### Usage

```console
rosrun ball_approach ball_approach_node
```

## core_msgs

A package for defining custom messages used in all codes.

ex) ball_position.msg



## data_integrate

Subscribing lidar(laser_scan) and camera data and do something

You need to change data type of lidar, topic name of both sensors, publishing cmd_vel type to use the node correctly.

ex) /scan -> /laser_scan, sensor_msgs::laser_scan -> sensor_msgs::PointCloud

#### Usage

```console
# data integrate
rosrun data_integrate data_integrate_node
# data show
rosrun data_integrate data_show_node
```



## robot_teleop

Nodes for manually manipulating the gripper and robot wheels.

#### Usage

```console
# gripper
rosrun robot_teleop prismatic_teleop_key
# wheel
rosrun robot_teleop wheel_teleop_key
```



## Tips

- If the prismatic joint fall down, check "Lock motor when target velocity is zero" in Joint Dynamic Properties in CoppeliaSim.
- Click "Toggle real-time mode" when you test your algorithms in simulator.
- ...



## Troubleshooting

- TBA

  

## Contact

Jonghwi Kim <stkimjh@kaist.ac.kr>

Haggi Do <kevindo@kaist.ac.kr>

Kyungseo Kim <chalseokim@kaist.ac.kr>
