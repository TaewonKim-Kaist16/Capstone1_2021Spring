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

The last test was done with modell F_5_6_4_kinect.ttm.
This model's camera view changes little bit for a long time.
Also, you can check map_ver3_with_model_F.ttt scene file.

## line_distance

This node needs opencv if you have not installed opencv follow below link.

https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html

Node for detecting planes with line and calculating relative distance and slope angle for them.

It publishes relative distances and slope angle of the planes as '/plane_info' topic.

Unit of slope is degree, and unit of distance is meter.

Slope and distance informations are sorted from nearest line to farthest line.

I tested it with line_detection camera(below view).

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


## coppeliasim_models

map_ver_x.ttt, my_robot_ver_x.ttm files

Map and my_robot can be updated later, so please check the notice board on klms regularly.

Just drag and drop files to coppeliasim window.

For reference, urdf file for my_robot is uploaded.



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
