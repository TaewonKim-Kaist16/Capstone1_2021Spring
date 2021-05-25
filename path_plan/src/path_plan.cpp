#include <ros/ros.h>
#include "string.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>

#include "localization/robot_position.h"
#include "localization/multi_position.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include <iostream>


using namespace std;

//in this node, ball harvesting mission will be controlled

//4-mode exist
int mode=1;

//define the required variables globally
//variable for updating information of obs,robot,red,green.
localization::robot_position robot_data;
localization::multi_position obstacle_data;
localization::multi_position red_balls_data;
localization::multi_position green_ball_data;

//to store pushing ball
localization::multi_position red_div;
red_div.num=0;
red_div.data.clear();

//pushing ball ball_number
int pushed_ball=0;

//for reset
localization::multi_position dummy,dummy1,dummy2;

//to remember the previous detected position
red_balls_data.data.resize(6);
red_balls_data.num=5;

obstacle_data.data.resize(4);
obstacle_data.num=4;

green_ball_data.num=1;
green_ball_data.data.x = 0;
green_ball_data.data.y = 0;

//current detected object number
int real_obstacle=0;
int real_red=0;
int green_detect=0;
int push_ball=0;

//for mode 1 function
//spin at fixed point
int turn=0;
//next step
bool finish =false;
int step =0;

//for mode 2
bool getob=false;
bool getdest=false;

//for velocity topic
float front_left;
float front_right;
float rear_left;
float rear_right;

//------------------------------------------------------------
//the other functions
//rearrange the point by increasing order with euclidean norm
void rearrange(localization::multi_position *jungsnumber)
{
  localization::multi_position hm=*jungsnumber;
  geometry_msgs::Point um;
  int a=hm.num;
  green_detect=0;
  //exception case : green ball detected as obstacle
  for (int i=0; i<a; i++)
  {
    if (pow((hm.data[i].x-green_ball_data.data.x),2)+pow((hm.data[i].y-green_ball_data.data.y),2)<0.5)
    {
      um = hm.data[i];
      hm.data[i]= hm.data[a-1];
      hm.data[a-1]=um;
      green_detect+=1;
    }
  }
  //main dish : rearrange by increasing order, if green ball does not detect, final one is (100,100)
  for (int i=0; i<a-2; i++)
  {
    for (int j=i+1; j<a-1; j++)
    {
      if (norm(&hm.data[i])>norm(&hm.data[j]))
      {
        um = hm.data[i];
        hm.data[i]=hm.data[j];
        hm.data[j]=um;
      }
    }
  }
}

//for remembering the object
int updating(localization::multi_position *dum,localization::multi_position *normal)
{
  geometry_msgs::Point p;
  int a=0;
  int b=0;
  localization::multi_position rec=*dum;
  localization::multi_position old=*normal;
  rearrange(&rec);
  b=green_detect;
  rearrange(&old);

  if (rec.num != old.num)
  {
      for (int i=0; i<rec.num; i++)
      {
        if (abs(old.data[i].x-rec.data[i].x)>0.1 && abs(old.data[i].y-rec.data[i].y)>0.1)
        {
          old.data[i].x=rec.data[i].x;
          old.data[i].y=rec.data[i].y;
        }
      }
  }
  else
  {
    for(int i=0; i<rec.num; i++)
    {
      if (abs(old.data[i].x-rec.data[i].x)>0.1 && abs(old.data[i].y-rec.data[i].y)>0.1)
      {
        old.data[i].x=rec.data[i].x;
        old.data[i].y=rec.data[i].y;
      }
    }
  }

  for (int i=0; i<old.num; i++)
  {
    if (old.data[i].x == 100 || old.data[i].x==50)
    {
      a++;
    }
  }
   b = a + abs(green_detect-b);
  *normal = old;
  return b;
}


//place green ball detection at last

//Calculate norm
float norm(geometry_msgs::Point *ha)
{
  float xm= ha.x;
  float ym= ha.y;
  xm=xm*xm+ym*ym;
  return xm;
}

//Calculate infimum_norm with x
float inf_norm(geometry_msgs::Point *a, geometry_msgs::Point *b)
{
  geometry_msgs::Point z = *a;
  geometry_msgs::Point r = *b;
  float dis= abs(z.x-r.x);
  return dis;
}

float inf_norm_y(geometry_msgs::Point *a)
{
  geometry_msgs::Point z = *a;
  float dis= abs(z.y-localization::robot_position.y);
  return dis;
}
//Calculate the norm with goal near walls
float bet_inf_norm(geometry_msgs::Point *a)
{
  float b;
  geometry_msgs::Point ref = *a;
  b=inf_norm(&ref, &green_ball_data.data);
  return b;
}

float bet_inf_norm_y(geometry_msgs::Point *a)
{
  float b;
  geometry_msgs::Point ref = *a;
  b=inf_norm_y(&ref);
  return b;
}

//Set vel_msg
void set_vel(float fl, float fr, float rl, float rr)
{
  float front_left = fl;
  float front_right = fr;
  float rear_left = rl;
  float rear_right = rr;
}

//--------------------------------------------
//define required function with mode


//for mode_1


//remembering the red ball
void redremember()
{
  int a;
  a = updating(&dummy1,&red_balls_data);
  real_red=a; //real detected ball count
}

//arround at position +-45degree
void spin_rem(int i)
{
  redremember();
  if (i==0)
  {
    set_vel(1,-1,1,-1);
  }
  else if (i==1)
  {
    set_vel(1,-1,1,-1);
  }
  else if (i==2)
  {
    set_vel(1,-1,1,-1);
  }
  else break;
}
//void change order of point
void change_order(geometry_msgs::Point *a, geometry_msgs::Point *b)
{
  geometry_msgs::Point c=*a;
  geometry_msgs::Point d=*b;
  geometry_msgs::Point du=d;
  du.x=c.x;
  c.x=d.x;
  d.x=du.x;
  du.y=c.y;
  c.y=d.y;
  d.y=du.y;
  *a=c;
  *b=d;
}

//calculate the order of mode_1
void order_red()
{
  std::vector<float> store(real_red);
  std::vector<float> dums(real_red, 0);
  int k=0;
  int p=real_red;
  for (int i=0; i<real_red; i++)
  {
    store(i)=bet_inf_norm(&red_balls_data.data[i]);
  }
  for (int j=0; j<real_red; j++)
  {
    if (store(j) <2.5)
    {
      dums(k)=store(j);
      store(j)=store(k);
      store(j)=dums(k);
      change_order(&red_position.data[j], &red_position.data[k]);
      k++;
    }
    else if (store(j)>=2.5)
    {
      dums(p-1)=store(j);
      store(j)=store(p-1);
      store(j)=dums(p-1);
      change_order(&red_position.data[j], &red_position.data[p-1]);
      p--;
    }
    //k is the number of pushing balls
    red_div.num=k;
    red_div.data.resize(k);
    for(int i=0; i<k; i++)
    {
      red_div.push_back(red_position.data[i]);
    }

    //reorder of balls have to be pushed
    for(int i=0; i<k-1; i++)
    {
      for(int j=i+1; j<k; j++)
      {
        float a,b;
        a=bet_inf_norm_y(&red_div.data[i]);
        b=bet_inf_norm_y(&red_div.data[j]);
        if(a>b)
        {
          change_order(&red_div.data[i],&red_div.data[j]);
        }
      }
    }
  }
}
//up to now, ball pushed along y axis



//main function for mode
//mode1
void mode1_act()
{
  if (finish == false) // detecting not finished
  {
    spin_rem(turn);
    if (robot_data.angle >= 90 && turn==0)
    {
      turn=1;
    }
    else if (robot_data.angle >=270 && turn ==1)
    {
      turn=2;
    }
    else if (robot_data.angle = 0 && turn==2)
    {
      set_vel(0,0,0,0); //stop
      finish = true; //next step
      turn=0; //reset for next use
      step=1;
    }
  }
  //up to now, the red ball detected, so divide whether push or not
  else if (step ==1)
  {
    order_red();
    step=2;
  }
  //
  else if (step ==2)
  {
    if(pushed_ball != red_div.num)
    {
      //autodriving; have to publish enough msg
      pushed_ball+=1;
    }
    else if (pushed_ball == red_div.num)
    {
      mode=2;
      step=0;
      finish = false;
    }
  }
}

void mode2_act()
{
  if(getob==false)
  {
    float x=real_obstacle.data[0].x;
    //calculate norm_y of obstacles
    for (int i=1; i<real_obstacle.num; i++)
      {
        if (real_obstacle.data[i].x< x)
        {
          x=real_obstacle.data[i].x;
        }
        getob=true;
      }
  }
  else if (getdest==false)
  {
    //autodriving to specific point, step will be chaged when arrive the destination
  }
  else if (finish == false)
  {
    spin_rem(turn);
    if (robot_data.angle >= 90 && turn==0)
    {
      turn=1;
    }
    else if (robot_data.angle >=270 && turn ==1)
    {
      turn=2;
    }
    else if (robot_data.angle = 0 && turn==2)
    {
      set_vel(0,0,0,0); //stop
      finish = true; //next step
      turn=0;
    }
  }
  else if (finish == true)
  {
    if (real_red == 5)
    {
      mode=4;
    }
    else mode=3;
  }
}

void mode3_act()
{

}

void mode4_act()
{

}


//---------------------------------------------------------------------------
//Path_plan Functions
void path_plan()
{
  switch(mode)
  {
    case 1:
    mode1_act();
    break;
    case 2:
    mode2_act();
    break;
    case 3:
    mode3_act();
    break;
    case 4:
    mode4_act();
    break;
  }
}

//---------------------------------------------------------------------------
//callback function
void obstacle_Callback(const localization::multi_position &obposition)
{
  dummy.data = obposition-> data;
  dummy.num = obposition -> num;
  if (dummy.num != 4)
  {
    dummy.data.resize(4);
    for (int i=3; i>dummy.num-1; i--)
    {
      dummy.data[i].x=100;
      dummy.data[i].y=100;
    }
  }
  real_obstacle=updating(&dummy,&obstacle_data);
}

void green_Callback(const localization::multi_position &gbposition)
{
  dummy1.data = gbposition-> data;
  dummy1.num = gbposition -> num;
    if (dummy1.num !=0)
    {
      green_ball_data.data.x = gbposition.data.x -> dummy1.data.x;
      green_ball_data.data.y = gbposition.data.y -> dummy1.data.y;
    }
}

void red_Callback(const localization::multi_position &rbposition)
{
  dummy2.data = rbposition -> data;
  dummy2.num = rbposition -> num;
  if (dummy2.num != 6)
  {
    dummy2.data.resize(6);
    for (int i=5; i>dummy.num-1; i--)
    {
      dummy2.data[i].x=50;
      dummy2.data[i].y=50;
    }
  }
}

void robot_Callback(const localization::robot_position &robotposition)
{
  robot_data.x = robotposition-> x;
  robot_data.y = robotposition-> y;
  robot_data.angle = robotposition-> angle;
}

//main function

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Path_plan_node");                                       //init ros nodd
    ros::NodeHandle nh;                                                              //create node handler
    ros::Subscriber sub_obstacle = nh.subscribe("map/data/obstacle", 1, obstacle_Callback); //create subscriber
    ros::Subscriber sub_red_ball = nh.subscribe("map/data/red_ball", 1, red_Callback); //create subscriber
    ros::Subscriber sub_green_ball = nh.subscribe("map/data/green_ball",1,green_Callback);
    ros::Subscriber sub_robot_geo = nh.subscribe("map/data/robot",1,robot_Callback);

    ros::Publisher pub_front_left_wheel= nh.advertise<std_msgs::Float64>("front_left_wheel_velocity", 10);
    ros::Publisher pub_front_right_wheel= nh.advertise<std_msgs::Float64>("front_right_wheel_velocity", 10);
    ros::Publisher pub_rear_left_wheel= nh.advertise<std_msgs::Float64>("rear_left_wheel_velocity", 10);
    ros::Publisher pub_rear_right_wheel= nh.advertise<std_msgs::Float64>("rear_right_wheel_velocity", 10);
    //just example for topic of wheel message
    while(ros::ok)
    {
      //declare the topic message
      std_msgs::Float64 front_left_wheel_msg;
      std_msgs::Float64 front_right_wheel_msg;
      std_msgs::Float64 rear_left_wheel_msg;
      std_msgs::Float64 rear_right_wheel_msg;

      path_plan();
      //set velocity
      front_left_wheel_msg.data=front_left;
      front_right_wheel_msg.data=front_right;
      rear_left_wheel_msg.data=rear_left;
      rear_right_wheel_msg.data=rear_right;
      //publish
      pub_front_left_wheel.publish(front_left_wheel_msg);   // publish left_wheel velocity
      pub_front_right_wheel.publish(front_right_wheel_msg);
      pub_rear_left_wheel.publish(rear_left_wheel_msg);   // publish left_wheel velocity
      pub_rear_right_wheel.publish(rear_right_wheel_msg);

      ros::spinOnce();
    }
    return 0;
}
