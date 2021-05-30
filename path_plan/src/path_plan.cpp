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
#include "path_plan/operation.h"
#include "path_plan/mode.h"


using namespace std;

//in this node, ball harvesting especially two points for translate
// Coordinate information : left lower corner is origin, x for horizontal, y for vertical, standard basis

//in this node, 4-mode exist
int mode=1;
//for receiving autodriving task
bool task=true;
bool spintask=false;
bool first= false;
int rotate=0;
bool approach = false;
//------------------------------------------------------------------------
//define the required variables globally
//variable for updating information of obs,robot,red,green.
localization::robot_position robot_data;
localization::multi_position obstacle_data;
localization::multi_position red_balls_data;
localization::multi_position green_ball_data;
geometry_msgs::Point goal_point;
geometry_msgs::Point sub_point;
localization::multi_position goal_position_rem;

// green ball initally detect
bool green_detect = false;
//To store pushing ball
localization::multi_position red_push;


//pushed ball ball_number
int pushed_ball=0;
int lack;
bool finish;
//for reset global variable  0-robot, 1-red, 2-green
localization::multi_position dummy,dummy1,dummy2;

//current detected object numbers
int real_obstacle=0;
int real_red=0;
int green_detect=0;
int push_ball=0;

//for mode 1 function -- maybe not use
//spin at fixed point
int turn=0;

//mode step
int wobblecount;
bool Wobble =false;
bool pushingstep =false;
bool dum =false;
bool dum1=false;
//for mode 2
bool getob=false;
bool getdest=false;

//for publishing message
path_plan::mode modes;
path_plan::operation operation;
//------------------------------------------------------------
//declare fuction part

//declare function for calculate euclidean norm
float norms(geometry_msgs::Point*);

//function for rearrange red or obstacles
void rearrange(locallization::multi_position*);

//to store red ball or obstacle
int updating(locallization::multi_position*,locallization::multi_position*);

//calculation of infinite norm (absolute value of y-direction)
float inf_norm_y(geometry_msgs::Point *);

//x direction
float inf_norm_x(geometry_msgs::Point *);

//Calculate the norm with two point
float bet_inf_norm_x(geometry_msgs::Point *, geometry_msgs::Point *);

float bet_inf_norm_y(geometry_msgs::Point *, geometry_msgs::Point *);

void change_order(geometry_msgs::Point *, geometry_msgs::Point *);

void set_desti(geometry_msgs::Point *);

void pushing_fun();

void sub_path(bool *);

vector<geometry_msgs::Point> disorderidentify(geometry_msgs::Point *);

float bet_norms(geometry_msgs::Point *, geometry::Point_msgs::Point *);

vector<int> search_area_obstacle(float , geometry_msgs::Point *, bool *);

void green_search();

vector<int> search_area_red(float, geometry_msgs::Point *, bool *);
//---------------------------------------------

//reset Functions
void reset()
{
  modes.mode.replace("NULL");
  modes.task=false;
  modes.spintask=false;
  modes.goal.clear();
  modes.angle=0;
  for(int i=0; i<2; i++)
  {
    operation.point[i].x=0;
    operation.point[j].y=0;
  }
}


//rearrange the point by increasing order with dictionary order 1 : y is larger, x is larger include all dummy value

void rearrange(localization::multi_position *jungsnumber)
{
  localization::multi_position hm=*jungsnumber;
  geometry_msgs::Point um;
  int a=hm.data.size();
  float c,d,p,q;

  //main dish : rearrange by increasing order, if green ball does not detect, final one is (100,100)
  for (int i=0; i<a-2; i++)
  {
    for (int j=i+1; j<a-1; j++)
    {
      c=inf_norm_y(&hm.data[i]);
      d=inf_norm_y(&hm.data[j]);
      if (c>d)
      {
        um = hm.data[i];
        hm.data[i]=hm.data[j];
        hm.data[j]=um;
      }
      else if (c==d)
      {
        p=inf_norm_x(&hm.data[i]);
        q=inf_norm_x(&hm.data[j]);
        if (p<q)
        {
          um = hm.data[i];
          hm.data[i]=hm.data[j];
          hm.data[j]=um;
        }
      }
    }
  }

  *jungsnumber = hm;
}

//for remembering the object (accumulate value for obstacle and spin mode of red ball)
int updating(localization::multi_position *dum,localization::multi_position *normal)
{
  geometry_msgs::Point p;
  int a=0;
  int rem=0;
  bool b=false;
  int c=0;
  int d=0;
  int count=0;
  localization::multi_position rec=*dum;
  localization::multi_position old=*normal;
  localization::multi_position new1;
  new1.data.clear();
  new1.num=old.num-1;
  rearrange(&rec);

  //green ball detect confirm mode
  if(green_ball_data.num !=0)
  {
    for (int i=0; i<old.num; i++)
    {
      p.x=abs(green_ball_data.data[i].x-rec.data[i].x);
      p.y=abs(green_ball_data.data[i].y-rec.data[i].y);
      if (p.x<0.2 && p.y<0.2 && (old.num-1 !=i))
      {
        b=true;
        rem=i;
        c=1;
      }

      if (rec.data[i].x==50 || rec.data[i].x==100)
      {
        //store dummy data
        d+=1;
      }
    }
  }

  if(b==true)
  {
    p=rec.data[old.num-1];
    rec.data[rem]=rec.data[old.num-1];
    rec.data[old.num-1]=p;
    // rearrange the order except dummy value
    for(int j=rem; j<old.num-d-2; j++)
    {
      chage_order(&rec.data[j],&rec.data[j+1]);
    }
  }

  //update the data
  for(int i=0; i<rec.num-c; i++)
  {
    for(int j=0; j<new1.num; j++)
    {
      p.x=abs(old.data[j].x-rec.data[i].x);
      p.y=abs(old.data[j].y-rec.data[i].y);
      if(p.x<0.2 && p.y<0.2)
      {
        new1.data.push_back(rec.data[i]);
        //initalize
        rec.data[i].x=0;
        rec.data[i].y=0;
        count+=1;
      }
    }
  }

  for(int i=0; i<rec.num-c; i++)
  {
    if((rec.data[i].x !=0) || (rec.data[i].x != 50) || (rec.data[i].x != 100))
    {
      new1.data.push_back(rec.data[i]);
      count+=1;
    }
  }
  rearrange(&new1);
  new1.num=old.num;
  new1.data.resize(old.num);
  for (int i=count; i<old.num; i++)
  {
    new1.data.push_back(1000);
  }

  *normal = old;
  return count;
}


//place green ball detection at last

//Calculate norm squared
float norms(geometry_msgs::Point *ha)
{
  geometry_msgs::Point z=*ha;
  float xm= z.x;
  float ym= z.y;
  xm=xm*xm+ym*ym;
  return xm;
}

float bet_norms(geometry_msgs::Point *ha, geometry::Point_msgs::Point *ma)
{
  geometry::Point_msgs::Point k=*ha;
  geometry::Point_msgs::Point m=*ma;
  float xm=k.x-m.x;
  float ym=k.y-m.y;
  xm=sqrt(xm*xm+ym*ym);
  return xm;
}
//Calculate infimum_norm with y
float inf_norm_y(geometry_msgs::Point *a)
{
  geometry_msgs::Point z = *a;
  float dis= abs(z.y);
  return dis;
}

float inf_norm_x(geometry_msgs::Point *a)
{
  geometry_msgs::Point z = *a;
  float dis= abs(z.x);
  return dis;
}
//Calculate the norm with two point w.r.t y axis
float bet_inf_norm_y(geometry_msgs::Point *a, geometry_msgs::Point *b)
{
  float c;
  geometry_msgs::Point ref = *a;
  geometry_msgs::Point ref1 = *b;
  c=abs(ref.y-ref1.y);
  return c;
}

//Calculate the norm with two point w.r.t x axis
float bet_inf_norm_x(geometry_msgs::Point *a, geometry_msgs::Point *b)
{
  float c;
  geometry_msgs::Point ref = *a;
  geometry_msgs::Point ref1 = *b;
  c=abs(ref.x-ref1.x);
  return c;
}


//--------------------------------------------
//define required function with mode


//for mode_1

//remembering the red ball
void redremember()
{
  int a;
  a = updating(&dummy2,&red_balls_data);
  real_red=a; //real detected ball count
}

//spin mode

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
  geometry_msgs::Point t,s;

  for (int i=0; i<real_red; i++)
  {
    store[i]=bet_inf_norm(&red_balls_data.data[i]);
  }
  for (int j=0; j<real_red; j++)
  {
    if (store[j] <2.5)
    {
      dums[k]=store[j];
      store[j]=store[k];
      store[k]=dums[k];
      change_order(&red_balls_data.data[j], &red_balls_data.data[k]);
      k++;
    }
    else if (store[j]>=2.5)
    {
      dums[p-1]=store[j];
      store[j]=store[p-1];
      store[j]=dums[p-1];
      change_order(&red_balls_data.data[j], &red_balls_data.data[p-1]);
      p--;
    }
    //k is the number of pushing balls
    red_push.num=k;
    red_push.data.resize(k);
    for(int i=0; i<k; i++)
    {
      red_push.data.push_back(red_balls_data.data[i]);
    }

    //reorder of balls have to be pushed
    for(int i=0; i<k-1; i++)
    {
      for(int j=i+1; j<k; j++)
      {
        t.y=red_push[i].y-robot_data.y;
        s.y=red_push[j].x-robot_data.y;
        if(t<s)
        {
          change_order(&red_push.data[i],&red_push.data[j]);
        }
      }
    }
  }
}
//up to now, ball pushed along y axis
void

//revise the goal point
void set_desti(geometry_msgs::Point *a)
{
  //input : initial goal value
  geometry_msgs::Point b= *a;
  vector<int> c,f,ha,ma;
  int d,g;
  float diff = b.x-robot_data.x;
  bool k=true;
  float r,m;
  ha.clear();
  ma.clear();

  if (diff<0)
  {
    k = true;
  }
  else k= false;

  c = search_area_obstacle(0.5, &b, &k);
  d=c.size();
  f= search_area_red(0.5, &b, &k);
  g=f.size();
  //up to now, all variable detect within the goal point w.r.t robot and goal position
  if (c+d !=0)
  {
    r= abs(goal_point.x-obstacle_data.data[c[0]].x);
    for (int i=1; i<d; i++)
    {
      if (abs(goal_point.x-obstacle_data.data[c[i]].x)<= 0.25)
      {
        ha.push_back(c[i]);
      }
    }
    for (int i=0; i<g; i++)
    {
      if (abs(goal_point.x-red_balls_data.data[f[i]].x)<= 0.25)
      {
        ma.push_back(f[i]);
      }
    }
  }
  int o=ma.size()+ha.size();

  if (c+d==0)
  {
    //no revise
  }
  else if (o != 0 && k==true)
  {
    goal_point.x+=0.25;
  }
  else if( o != 0 && (k==false))
    {
      goal_point.x-= 0.25s;
    }
}

//searching the area near the goal_point
vector<int> search_area_obstacle(float a, geometry_msgs::Point *b, bool *k)
{
  float r= *a;
  geometry_msgs::Point z= *b;
  bool left=true;
  bool e = *k;


  vector<int> storage;
  storage.clear();
  float b,c;

  for (int i=0; i<real_obstacle; i++)
  {
    t = goal_point.x-obstacle_data[i].x;
    if (t>0)
    {
      left = false;
    }
    else left = true;

    b = bet_inf_norm_y(&obstacle_data[i],&z);
    if (b<r && (left == e))
    {
        storage.push_back(i)
    }
  }
  //return near obstacle index
  return storage;
}

//in near goal position, red ball count
vector<int> search_area_red(float a, geometry_msgs::Point *b, bool *k)
{
  float r= *a;
  geometry_msgs::Point z= *b;
  bool e = *k;
  bool left= false;

  vector<int> storage;
  storage.clear();
  float b,c;

  for (int i=0; i<real_red; i++)
  {
    t= goal_point.x-red_balls_data[i].x;
    if (t<0)
    {
      left= true;
    }
    else left = false;

    b = bet_inf_norm_y(&red_balls_data[i],&z);
    if (b<r && left == e)
    {
        storage.push_back(i)
    }
  }
  //return near obstacle index
  return storage;
}

void green_search()
{
  geometry_msgs::Point p;
  p.x=0.75;
  p.y=robot_data.data.y;
  if (dum1 == false && rotate ==2)
  {
    set_desti(&p);
    goal_position_rem.push_back(goal_point);
    dum1=true;
  }

  if (dum1 == true && dum == false)
  {
    sub_path(&dum);
  }
  if (dum==true && rotate ==2)
  {
    modes.mode.replace("Spin");
    modes.goal.x=2.5;
    modes.goal.y=5;
    modes.task=false;
  }
  else if (task == true)
  {
    rotate=3;
    dum=false;
    dum1=false;
  }
}


//return inline_path obstacle & ball information
vector<geometry_msgs::Point> disorderidentify(geometry_msgs::Point *p)
{
  geometry_msgs::Point a = *p;
  vector<geometry_msgs::Point> b,c;
  geometry_msgs::Point reorder,dump;

  b.clear();
  c.clear();

  vector<int> a= search_area(z);
  //calculate line
  float m= (a.y-robot_data.x)/(a.x-robot_data.x);
  float n = robot_data.y-((robot_data.x)/m);
  for (int i=0; i<real_red; i++)
  {
    if (abs(red_balls_data.data[i].y -red_balls_data.data[i].x*m-n)<0.35)
    {
      b.push_back(red_balls_data[i]);
    }
  }
  for( int i=0; i<z.size(); i++)
  {
    //search whether overlay with path
    if (abs(obstacle_data.data[z[i]].y -obstacle_data.data[z[i]].x*m-n)<0.35)
    {
      b.push_back(obstacle_data.data[z[i]]);
    }
  }

  for(int i=0; i<b.size(); i++)
  {
    c.push_back(abs(b[i].x-robot_data.x));
  }
  //reorder with first : close to robot w.r.t x axes
  for (int i=0; i<b.size()-1; i++)
  {
    reorder.x=c[i].x;
    reorder.y=c[i].y;
    for (int j=i+1; j<b.size(); j++)
    {
      if(c[j].x<reorder.x)
      {
        dump.x=b[j].x;
        b[j].x=b[i].x;
        b[i].x=dump.x;
        dump.y=b[j].y;
        b[j].y=b[i].y;
        b[i].y=dump.y;
      }
    }
  }

  return b;
}

void sub_path(bool *a)
{
  //trigger for loop until task done
  bool z =*a;
  vector<geometry_msgs::Point> r = disorderidentify(&goal_point);
  int s =r.size();
  if (s == 0)
  {
    operation[1].x=goal_point.x;
    operation[1].y=goal_point.y;
    *a=true;
    goal_point.clear();
  }
  if(task == true && z == false)
  {
    if (s != 0 && r.y-1.0 >0.9)
    {
      operation[1].x=r.x;
      operation[1].y=r.y-1.0;

    }
    else if (s != 0 && r.y-1.0<0.9)
    {
      operation[1].x=r.x;
      operation[1].y=r.y+1.0;
    }
    modes.mode.replace("Go");
     modes.task = false;
  }
}
//
void pushing_fun()
{
  if (approach == false)
  {
    operation[1].x=red_push.data[red_push.num-1].x;
    if (inf_norm_y(&red_push.data[red_push.num-1])<0.5)
      {
        operation[1].y=red_push.data[red_push.num-1].y+0.4;
      }
    else operation[1].y=red_push.data[red_push.num-1].y-0.4;
    modes.mode.replace("Go");
    modes.task=false;
    approach=true;
  }
  else
  {
    operation[1].x=red_push.data[red_push.num-2].x;
    opeartion[1].y=red_push.data[red_push.num-2].y;
    red_push.data.pop_back();
    red_push.resize(red_push.num-1);
    modes.mode.replace("Go");
    modes.task=false;
    approach = false;
  }
}

//main function for mode
//mode1
void mode1_act()
{
  //Spin at first position 90deg
  if (rotate== 0)
  {
    modes.mode.replace("Spin");
    if(first==false)
      {
        //90 turn
        modes.goal.x=0;
        modes.goal.y=robot_data.y;
        modes.task=false;
        first=true;
      }
    redremember();
    if(task == true)
    {
      rotate=1;
    modes.mode.replace("NULL");
      task=false;
      first=false;
    }
  }

  else if (wobblecount == 0 && Wobble==false)
  {
    redremember();
    modes.mode.replace("Wobble");
    modes.spintask=false;
    modes.angle=20;
    if(spintask==true)
    {
      wobblecount=1;
    }
  }

  else if (wobblecount==1)
  {
    //go to middle points
    redrember();
    modes.mode.replace("Go");
    modes.task=false;
    if (first == false)
    {
      //set destination
      goal_point.x=2.5;
      goal_point.y=robot_data.data.y;
      first = true;
      set_desti(&goal_point);
    }
    else sub_path();
    if((robot_data.x-goal_point.x)<-0.2)
    {
      goal_point.clear();
      wobblecount=2;
    modes.mode.replace("NULL");
      modes.task=false;
      goal_point.clear();
    }
  }
  else if(rotate==1)
  {
    modes.mode.replace("Spin");
    if (first == false)
    {
      modes.goal.x=robot_data.x;
      modes.goal.y=5;
      modes.task=false;
      first=true;
    }
    redremember();
    if (task==true)
    {
      rotate=2;
    modes.mode.replace("NULL");
      task=false;
      first=false;
    }
  }
  else if (wobblecount==2)
  {
    redremember();
    modes.mode.replace("Wobble");
    modes.spintask=false;
    modes.angle=20;
    if (spintask==true)
    {
      wobblecount=0;
      Wobble=true;
    }
  }

  else if (green_detect==false)
  {
    green_search();
  }
  //up to now, scan the red ball finished and green ball coordinate obtained
  else if (pushingstep ==false)
  {
    order_red();
    pushingstep=true;

  }
  //pushing stage
  if (pushingstep ==true)
  {
    if(task==true)
    {
      pushing_fun();
      modes.task=false;
    }

    if (red_push.num== 0)
    {
      mode=2;
      rotate=0;
      modes.task=false;
    }
  }
}

void mode2_act()
{
  lack=0;
  if(getob==false)
  {
    geometry_msgs::Point p = 0.5*(obstacle_data.data[0]+obstacle_data.data[1]);
    p.y= 0.2*p.y+0.8*obstacle_data.data[3].y;
    p.x=obstacle_data.data[2].x-0.2;
    set_desti(&p);
    getob=true;
  }
  else if (getdest==false)
  {
    sub_path(&dum);
  }
  else if (finish == false && task==true)
  {
    redremember();
    modes.angle=30;
    modes.mode.replace("Spin");
    modes.goal.x=robot_data.x-0.1;
    modes.goal.y=robot_data.y-0.1;
    modes.task=false;
  }
  else if (finish == true)
  {
    for(int i=0; i=real_red; i++)
    {
        if (red_balls_data.data[i].x == 50 || red_balls_data.data[i].x == 100 || red_balls_data.data[i].x == 1000)
        {
          lack+=1;
        }
    }
    if (real_red == 5)
    {
      mode=4;
    }
    else mode=3;
  }
}

void mode3_act()
{

  if (task == true && dum == false)
  {
    set_desti(&goal_position_rem[goal_position_rem.size()-1]);
    modes.task=false;
  }
  else if (dum == true)
  {
    modes.mode.replace("Go");
    sub_path(&dum1);
  }
  else if (dum1 == true)
  {
    modes.mode.replace("Wobble");
    modes.angle = 30;
  }
  if (dummy1.num !=0)
  {
    set_desti(&dummy1.data[0]);

  }

  else mode = 4;
}

void mode4_act()
{

}


//---------------------------------------------------------------------------
//Path_plan Functions
void pathplan()
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

//obstacle call back function only for current value
void obstacle_Callback(const localization::multi_position &obposition)
{
  dummy.data = obposition.data;
  dummy.num = obposition.num;
  if (dummy.num != 4)
  {
    dummy.data.resize(4);
    //fill with 100 very large value for no detected one
    for (int i=3; i>dummy.num-1; i--)
    {
      dummy.data[i].x=100;
      dummy.data[i].y=100;
    }
  }
  //store the accumulated obstacle number
  real_obstacle=updating(&dummy,&obstacle_data);
}

//green ball call back function only for current value
void green_Callback(const localization::multi_position &gbposition)
{
  dummy1.data = gbposition.data;
  dummy1.num = gbposition.num;
    if (dummy1.num !=0)
    {
      green_detect=true;
      for(int i=0; i<green_ball_data.num; i++)
      {
        green_ball_data.data[i].x = dummy1.data[i].x;
        green_ball_data.data[i].y = dummy1.data[i].y;
      }
    }
}

//red_ball call_back function only for current value
void red_Callback(const localization::multi_position &rbposition)
{
  dummy2.data = rbposition.data;
  dummy2.num = rbposition.num;
  if (dummy2.num != 6)
  {
    //final index for case dectecting green ball
    dummy2.data.resize(6);
    //fill with dummy value for not detect
    for (int i=5; i>dummy.num-1; i--)
    {
      dummy2.data[i].x=50;
      dummy2.data[i].y=50;
    }
  }
}

//robot_position call back function(only current value)
void robot_Callback(const localization::robot_position &robotposition)
{
  robot_data.x = robotposition.x;
  robot_data.y = robotposition.y;
  robot_data.angle = robotposition.angle;
}

void task_done(const path_plan::mode &t)
{
  t -> task =  modes.task;
  t -> spintask = modes.spintask;
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
    ros::Subscriber sub_task = nh.subscribe("path_plan/mode",1,task_done);
    //subscribe node of autodriving

    ros::Publisher pub_point = nh.advertise<path_plan::operation>("path_plan/data",1);
    ros::Publisher pub_mode = nh.advertise<path_plan::mode>("path_plan/wheel_mode",1);

    bool task=true;
    bool spintask=false;
    bool first= false;
    int rotate=0;
    bool approach = false;

    int wobblecount;
    bool Wobble =false;
    bool pushingstep =false;
    bool dum =false;
    bool dum1=false;
    //for mode 2
    bool getob=false;
    bool getdest=false;

    red_push.num=0;
    red_push.data.clear();

    red_balls_data.data.resize(6);
    red_balls_data.num=5;

    obstacle_data.data.resize(4);
    obstacle_data.num=4;
    goal_position_rem.clear();
    green_ball_data.num=1;
    for(int i=0; i<green_ball_data.num; i++)
    {
      green_ball_data.data[i].x = 0;
      green_ball_data.data[i].y = 0;
    }

    modes.mode.replace("NULL");
    modes.angle=0;
    modes.goal.x=0;
    modes.goal.y=0;

    operation.point.resize(2);
    operation.point.clear();

    //just example for topic of wheel message
    while(ros::ok)
    {
      //declare the topic message
      if(task ==true && mode !=1)
      {
        pathplan();
      }
      //set message
      pub_mode.publish(mode);
      pub_point.publish(operation);
      reset();
      ros::spinOnce();
    }
    return 0;
}
