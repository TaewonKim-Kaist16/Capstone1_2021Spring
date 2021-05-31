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

int mode=1;

//-------------------------------------ball & obstacle--------------------
localization::robot_position robot_data; //reiceive robot position
localization::multi_position obstacle_data; // receiving the obstacle data
localization::multi_position red_balls_data; // receive the red ball deta during ball remember
localization::multi_position green_ball_data; // green ball update


//----------------------------------------------
bool green_detect = false;
bool task_done = false;
//To store pushing ball
localization::multi_position red_push;
localization::multi_position not_push;

geometry_msgs::Point destination;

int red_detected=0;
int score=0;
//for reset global variable  0-robot, 1-red, 2-green
localization::multi_position dummy,dummy1,dummy2;

//current detected object numbers
int real_obstacle=0;
int real_red=0;
int push_ball=0;


//store the infor of ob, balls near the destination point
vector<geometry_msgs::Point> near_obstacle;
vector<geometry_msgs::Point> near_red;
vector<int> subcoordinate(4,0);
vector<geometry_msgs::Point> inpathob;
vector<geometry_msgs::Point> sub_destination;
int sub_dest_num = 0;
//for mode 1 function -- maybe not use
//spin at fixed point
int turn=0;

//mode step
int mode_1_num=1;
int mode_2_num=1;
int mode_3_num=1;
int mode_4_num=1;

//for publishing message
//path_plan::mode modes;
//path_plan::operation operation;

//---------------------declare function-----------------------------------
  // distance function
float norms(geometry_msgs::Point );
float bet_norms(geometry_msgs::Point , geometry_msgs::Point );
float inf_norm_y(geometry_msgs::Point );
float inf_norm_x(geometry_msgs::Point );
float bet_inf_norm_y(geometry_msgs::Point , geometry_msgs::Point );
float bet_inf_norm_x(geometry_msgs::Point , geometry_msgs::Point );
float bet_dis_robot(geometry_msgs::Point *, localization::robot_position *);

  // deal with point
void reset(localization::multi_position *);
void change_order(geometry_msgs::Point * , geometry_msgs::Point *);
void rearrange(localization::multi_position *);
void arrange_x(localization::multi_position *);
void arrange_y(localization::multi_position *);
int updating(localization::multi_position *, localization::multi_position *);
void rememberred(localization::multi_position );

void select_ball();

void line_clean();
void sub_desti();
void sub_decision();
float calculate_theta(geometry_msgs::Point *, geometry_msgs::Point *);
void infor_coor(float *);
void near_point(geometry_msgs::Point *);
float orientation(geometry_msgs::Point *);
void set_point_revise(geometry_msgs::Point *);
int search_al(int , bool );
void get_des();
void choose_point(geometry_msgs::Point *);

bool select_throw(geometry_msgs::Point *, localization::multi_position *, int );

geometry_msgs::Point meanob();

  //motion
void wobblefun();
void pushingfun();
  //path plan
void pathplan();
void mode1_act();
void mode2_act();
void mode3_act();
void mode4_act();
//------------------------------------------------------------------------

void reset(localization::multi_position *d)
{
  localization::multi_position p = *d;
  p.num = 0;
  p.data.clear();
  *d=p;
}

//wobble motion with red ball remember
void wobblefun()
{
  rememberred(dummy2);
}

void rememberred(localization::multi_position a)
{
  real_red = updating(&a,&red_balls_data);
}
  //rearragne by dictionary order, y ,x  large y at last, large x is last --- finished
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
      c=inf_norm_y(hm.data[i]);
      d=inf_norm_y(hm.data[j]);
      if (c>d)
      {
        um = hm.data[i];
        hm.data[i]=hm.data[j];
        hm.data[j]=um;
      }
      else if (c==d)
      {
        p=inf_norm_x(hm.data[i]);
        q=inf_norm_x(hm.data[j]);
        if (p>q)
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

void arrange_x(localization::multi_position *jungsnumber)
{
  localization::multi_position a = *jungsnumber;
  geometry_msgs::Point p;
  int size = a.data.size();
  a.num = size;
  //rearrange with large - small x
  for (int i=0; i<size-1; i++)
  {
    for (int j=i+1; j<size; j++)
    {
      if (abs(a.data[i].x) < abs(a.data[j].x) )
      {
        p = a.data[j];
        a.data[j]=a.data[i];
        a.data[i] = p;
      }
    }
  }
  *jungsnumber = a;
}

void arrange_y(localization::multi_position *jungsnumber)
{
  localization::multi_position a = *jungsnumber;
  geometry_msgs::Point p;
  int size = a.data.size();
  a.num = size;
  //rearrange with large - small x
  for (int i=0; i<size-1; i++)
  {
    for (int j=i+1; j<size; j++)
    {
      if (abs(a.data[i].y) < abs(a.data[j].y) )
      {
        p = a.data[j];
        a.data[j]=a.data[i];
        a.data[i] = p;
      }
    }
  }
  *jungsnumber = a;
}

int updating(localization::multi_position *dum,localization::multi_position *normal)
{
  geometry_msgs::Point p;
  int a=0;
  int rem=0;

  //green_ball same data exist
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
  if(green_detect==true)
  {
    for (int i=0; i<old.num; i++)
    {
      p.x=abs(green_ball_data.data[i].x-rec.data[i].x);
      p.y=abs(green_ball_data.data[i].y-rec.data[i].y);
      //if green ball does not exist at final position
      if ((p.x<0.2 && p.y<0.2) && (old.num-1 !=i))
      {
        //have to change order
        b=true;
        //remember the index
        rem=i;
        c=1;
      }
      //for dummy data which filled at no real data exist
      if (rec.data[i].x==50 || rec.data[i].x==100)
      {
        //count the number
        d+=1;
      }
    }
  }
  //green ball reach its proper position
  if(b==true)
  {
    p=rec.data[old.num-1];
    rec.data[rem]=rec.data[old.num-1];
    rec.data[old.num-1]=p;

    // rearrange the order except dummy value
    for(int j=rem; j<old.num-d-2; j++)
    {
      change_order(&rec.data[j],&rec.data[j+1]);
    }
  }
  //up to now, green ball on the last position

  //update the data
  for(int i=0; i<rec.num-c; i++)
  {
    for(int j=0; j<new1.num; j++)
    {
      p.x=abs(old.data[j].x-rec.data[i].x);
      p.y=abs(old.data[j].y-rec.data[i].y);

      //if data same, update with recent data
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
  //fill other recent value
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

  geometry_msgs::Point u;
  u.x=1000; u.y=1000;

  for (int i=count; i<old.num; i++)
  {
    new1.data.push_back(p);
    new1.data.push_back(p);
  }

  *normal = old;
  return count;
}

//------------------------distance function ----------------------
float norms(geometry_msgs::Point ha)
{
  float xm= ha.x;
  float ym= ha.y;
  xm=sqrt(xm*xm+ym*ym);
  return xm;
}

float bet_norms(geometry_msgs::Point ha, geometry_msgs::Point ma)
{
  float xm=ha.x-ma.x;
  float ym=ha.y-ma.y;
  xm=sqrt(xm*xm+ym*ym);
  return xm;
}
//Calculate infimum_norm with y
float inf_norm_y(geometry_msgs::Point a)
{
  float dis= abs(a.y);
  return dis;
}

float inf_norm_x(geometry_msgs::Point a)
{
  float dis= abs(a.x);
  return dis;
}
//Calculate the norm with two point w.r.t y axis
float bet_inf_norm_y(geometry_msgs::Point a, geometry_msgs::Point b)
{
  float c;
  c=abs(a.y-b.y);
  return c;
}

//Calculate the norm with two point w.r.t x axis
float bet_inf_norm_x(geometry_msgs::Point a, geometry_msgs::Point b)
{
  float c;
  c=abs(a.x-b.x);
  return c;
}

//calculate robot and destination point
float bet_dis_robot(geometry_msgs::Point *a, localization::robot_position *b)
{
  geometry_msgs::Point p = *a;
  localization::robot_position q = *b;
  float result;
  float x_diff = p.x-q.x;
  float y_diff = p.y-q.y;
  result = sqrt(x_diff*x_diff + y_diff* y_diff);
  return result;
}

//change of two point = finished
void change_order(geometry_msgs::Point *a, geometry_msgs::Point *b)
{

  geometry_msgs::Point du1 = *a;
  geometry_msgs::Point du2 = *b;
  geometry_msgs::Point du;

  du.x=du1.x;
  du1.x=du2.x;
  du2.x=du.x;
  du.y=du1.y;
  du1.y=du2.y;
  du2.y=du.y;
  *a=du1;
  *b=du2;
}

void line_clean()
{
  inpathob.clear();
  float m = (destination.x-robot_data.x)/(destination.y-robot_data.y);

  float b = robot_data.y-m*robot_data.x;

  float vertdis;

  for (int i=0; i<real_obstacle; i++)
  {
    vertdis = abs(m*obstacle_data.data[i].x+obstacle_data.data[i].y)/(m*m+1);
    if (vertdis <= 0.3)
    {
      inpathob.push_back(obstacle_data.data[i]);
    }
  }

  for (int i=0; i<real_red; i++)
  {
    vertdis = abs(m*red_balls_data.data[i].x+red_balls_data.data[i].y)/(m*m+1);
    if (vertdis <= 0.3)
    {
      inpathob.push_back(red_balls_data.data[i]);
    }
  }
}
//sub_destination decision == finish
void sub_desti()
{
  line_clean();
  if (inpathob.size() == 0)
  {
    sub_destination.resize(1);
    sub_destination[0].x = destination.x;
    sub_destination[0].y = destination.y;
  }
  else
  {
    sub_decision();
  }
}

void sub_decision()
{
  localization::multi_position a;
  a.data.clear();
  float du1,du2;

  int size = inpathob.size();
  a.data.resize(size+1);
  du1= inf_norm_y(a.data[0]);
  du2= inf_norm_y(a.data[1]);

  for (int i=0; i<size-1; i++)
  {
      a.data[i].x = inpathob[i].x-robot_data.x;
      a.data[i].y = inpathob[i].y-robot_data.y;
  }
  arrange_x(&a);
  a.data.push_back(destination);
  float dist = bet_norms(a.data[0], a.data[1]);


  if (dist > 0.5)
  {
      a.data[0].x= a.data[0].x+robot_data.x;
      if (a.data[0].y>0)
      {
        a.data[0].y = a.data[0].y+robot_data.y-0.45;
        if (du1 -0.45< 0.2)
        a.data[0].y+=1;
      }
      else
      {
        a.data[0].y = a.data[0].y+robot_data.y+0.45;
        if (du1 +0.45 >5.2)
        {
          a.data[0].y-=1;
        }
      }
  }
  else
  {
    a.data[0].x= a.data[0].x+robot_data.x;
    if (a.data[0].y>0)
    {
      a.data[0].y = a.data[1].y+robot_data.y-0.45;
      if (du2 -0.45< 0.2)
      {
        a.data[0].y+=1;
      }
    }
    else {
      a.data[0].y = a.data[1].y+robot_data.y+0.45;
      if (du2 -0.45< 0.2)
      {
        a.data[0].y-=1;
      }
    }
  }
  sub_destination.push_back(a.data[0]);

  //up to now, rearrange with large to small w.r.t x (relative to robot)
  while (size== 0)
  {
    sub_decision();
  }


}

void choose_point(geometry_msgs::Point *a)
{
  geometry_msgs::Point p = *a;
  set_point_revise(&p);
  sub_desti();
  sub_dest_num = sub_destination.size();
  //if this function excuted, sub_destination filled
}

//choose pushing ball
void select_ball()
{
  red_push.data.clear();
  red_push.num=0;
  not_push.num=0;
  not_push.data.clear();

  for (int i=0; i<real_red; i++)
  {
      float du = inf_norm_y(red_balls_data.data[i]);
      if (du < 2.5)
      {
        red_push.num +=1;
        red_push.data.resize(red_push.num);
        red_push.data.push_back(red_balls_data.data[i]);
      }
      else
      {
        not_push.num +=1;
        not_push.data.resize(not_push.num);
        not_push.data.push_back(red_balls_data.data[i]);
      }
  }
  arrange_y(&red_push);
  arrange_x(&not_push);

}
//to calculate radian between initial destination point and near obstacle -- finish
float calculate_theta(geometry_msgs::Point *a, geometry_msgs::Point *b)
{
  geometry_msgs::Point p =*a;
  geometry_msgs::Point q =*b;

  float x_diff = p.x-q.x;
  float y_diff = p.y-q.y;

  float co = x_diff/bet_norms(p, q);
  float si = y_diff/bet_norms(p, q);
  float theta;

  if (si>=0)
  {
    theta = acos(co);
  }
  else theta = 2*M_PI-acos(co);

  return theta;
}

//based on radian, coordiate info update -- finish
void infor_coor(float *a)
{
  float theta = *a;
  if (0<=theta <0.5*M_PI )
  {
    subcoordinate[0]+=1;
  }
  else if(theta <= M_PI)
  {
    subcoordinate[1]+=1;
  }
  else if (theta <= 1.5*M_PI)
  {
    subcoordinate[2]+=1;
  }
  else subcoordinate[3]+=1;
}

//get near ob&red and coordinate information -- finish
void near_point(geometry_msgs::Point *a)
{
  geometry_msgs::Point p = *a;
  near_obstacle.clear();
  near_red.clear();

  //calculate norm so that filter the values
  for (int i=0; i<real_obstacle; i++)
  {
    float b = bet_norms(obstacle_data.data[i], p);
    if (b<=0.3)
    {
      near_obstacle.push_back(obstacle_data.data[i]);
      infor_coor(&b);
    }
  }

  for (int i=0; i<real_red; i++)
  {
    float b = bet_norms(red_balls_data.data[i], p);
    if (b<=0.3)
    {
      near_red.push_back(red_balls_data.data[i]);
      infor_coor(&b);
    }
  }
}

// calculate theta between robot and destination point --finish
float orientation(geometry_msgs::Point *a)
{
  geometry_msgs::Point p = *a;
  float x = acos((p.x-robot_data.x)/bet_dis_robot(&p, &robot_data));
  float y = asin((p.y-robot_data.y)/bet_dis_robot(&p, &robot_data));
  float theta;

  if (y>=0)
  {
    theta = acos(x);
  }
  else theta = 2*M_PI-acos(x);

  return theta;
}


//decide left or right based on radian -- finish
bool lr(float *a)
{
  float theta = *a;
  bool left;
  if (M_PI/2<=theta<=3*(M_PI/2))
  {
    left =true;
  }
  else left = false;
}

//for publshing to sub_destination
void get_des()
{
  if (task_done == true)
  {
    sub_dest_num-=1;
    //publishing function
  }

  if (sub_dest_num == 1)
  {
    if(task_done == true)
    {
      destination.x=0;
      destination.y=0;
    }
  }
}

bool select_throw(geometry_msgs::Point *a, localization::multi_position *b, int k)
{
  geometry_msgs::Point p = *a;
  localization::multi_position q = *b;
  localization::multi_position t = not_push;
  int dum;
  int min_ob, max_ob;
  q.data.resize(t.num+q.num);

  for(int i=0; i<not_push.num; i++)
  {
    q.data.push_back(not_push.data[i]);
  }

  bool same;

  arrange_x(&q);
  for(int i=0; i<q.data.size(); i++)
  {
    if(q.data[i].x>2.3)
    {
      min_ob =i-1;
      break;
    }
  }


  for(int i=q.data.size()-1; i=0; i--)
  {
    if(q.data[i].x<2.7)
    {
      max_ob = i+1;
      break;
    }
  }

  for (int i=0; q.data.size()-1; i++)
  {
    if (p.x == q.data[i].x)
    {
      same = true;
      break;
    }
    else same = false;
  }
  if (same = true)
  {
    if (q.data[min_ob].x<p.x<q.data[max_ob].x)
    {
      if (p.x <2.5)
      {
        p.x = 0.5*(p.x+q.data[min_ob].x);
      }
      else p.x = 0.5*(p.x+q.data[max_ob].x);
    }
    else
    {
      for (int i=0; i<q.num; i++)
      {
        if (q.data[i].x> p.x)
        {
          dum = i-1;
          break;
        }
      }
      if( p.x < 2.5)
      {
        p.x = 0.5*(q.data[dum].x+q.data[dum+1].x);
      }
      else
      {
        p.x = 0.5*(q.data[dum-1].x+q.data[dum].x);
      }
    }
  }
  *a=p;
  return same;
}

void pushingfun()
{
  localization::multi_position a = obstacle_data;
  int z = 0;
  for (int i=0; i<red_push.num; i++)
  {
    float d;
    geometry_msgs::Point p = red_push.data[i];
    geometry_msgs::Point q;
    choose_point(&p);
    get_des();

    while (task_done == false)
    {
      //publishing the message which is global
      //get near the ball
    }
    //spin() to align


    if (red_push.num-1 == i)
    {
      d=  bet_inf_norm_y(p, red_push.data[i+1]);
    }
    else d = 1;
    arrange_x(&a);
    bool change = select_throw(&p, &a, i);
    if (change == false)
    {
      q.x = p.x;
      q.y = robot_data.y+d;
      int z=0;
    }
    else
    {
      q.x = p.x;
      q.y = robot_data.y;
      z=1;
    }
    choose_point(&q);

    while(z>=0)
    {
      get_des();
      if (task_done == true && change ==false)
      {
        z=-1;
      }
      else if (change ==true)
      {
        q.y=d;
        z=0;
        choose_point(&q);
      }
    }
  }
}

//set_point is revised becaused of close obstacle -- finish
void set_point_revise(geometry_msgs::Point *a)
{
  geometry_msgs::Point p = *a;
  //calculate theta and orientaiton w.r.t robot
  float theta = orientation(&p);
  bool left = lr(&theta);
  int num =0;
  int actual;
  int ob = near_obstacle.size();
  int red = near_red.size();

  //w.r.t destination point
  theta =2*M_PI-theta;

  if (0<=theta <0.5*M_PI )
  {
    num=1;
  }
  else if(theta <= M_PI)
  {
    num=2;
  }
  else if (theta <= 1.5*M_PI)
  {
    num=3;
  }
  else num = 4;

  //maximum is 4 if it is larger than 4 it's good anothoer point
  near_point(&p);
  if ((ob+red) != 0)
  {
    //up to now, get a ob&red, and coordinate infor_coor
    actual = search_al(num, left);
    if (actual == 5)
    {
      if (left == true)
      {
        p.x = p.x+0.2;
      }
      else{
        p.x= p.x-0.2;
      }
      set_point_revise(&p);
    }
    if (left = true)
      {
        switch (actual)
        {
          case 1:
          p.x+= 0.2;
          break;
          case 2:
          p.y+=0.2;
          break;
          case 3:
          p.y-=0.2;
          break;
          case 4:
          p.x+=0.2;
          break;
        }
      }
    else
    {
      switch (actual)
      {
        case 1:
        p.y+= 0.2;
        break;
        case 2:
        p.x-=0.2;
        break;
        case 3:
        p.x-=0.2;
        break;
        case 4:
        p.y-=0.2;
        break;
      }
    }
  }
  *a=p;
}

// searching alternative coordinate -- finish
int search_al(int i, bool left)
{
  int a;
  int plus = (i+1)%4;
  int minus = (i-1)%4;
  int factor;
  if (i <2)
  {
    factor=1;
  }
  else factor =-1;

  if(subcoordinate[i-1] == 0)
  {
    a = i;
  }
  else
  {
    if (factor >0)
    {
      if (subcoordinate[plus-1] == 0)
      {
        a=plus;
      }
      else if(subcoordinate[minus-1] == 0)
      {
        a=minus;
      }
      else if (subcoordinate[plus] == 0)
      {
        a=(i+2)%4;
      }
      else a =5;
    }
    else
    {
      if (subcoordinate[minus-1] == 0)
      {
        a=minus;
      }
      else if(subcoordinate[plus-1] == 0)
      {
        a=plus;
      }
      else if (subcoordinate[plus] == 0)
      {
        a=(i+2)%4;
      }
      else a =5;
    }
  }
  return a;
}

//mode1_act function
void mode1_act()
{

  switch (mode_1_num)
  {
    case 1:
  //1. wabble at start point with red ball remember
    {
      wobblefun();
    //publish
      if (task_done == true)
      {
        geometry_msgs::Point p;
        p.x = 2.5;
        p.y = robot_data.y;
        choose_point(&p);
        //publish task_done false
      }
      break;
    }

    case 2:
  //2. move to another point
    {
      rememberred(dummy2);
      float du = bet_dis_robot(&destination,&robot_data);
      get_des();
      if (task_done == true && du <0.02)
      {
        mode_1_num = 3;
      }
      break;
    }

    case 3:
  //3. wobble at current point
    {
      wobblefun();
      if (task_done == true)
      {
        mode_1_num = 4;
      }
      break;
    }

    case 4 :
    //4. calculate the information of ball have to be pushed
    {
      select_ball();
      mode_1_num = 5;
      break;
    }

  //5. pushing the ball one by one
    case 5:
    {
      pushingfun();
      if (task_done == true)
      {
        mode=2;
        red_balls_data.data.clear();
      }
      break;
    }
  }

}


geometry_msgs::Point meanob()
{
  geometry_msgs::Point p;
  for (int i=0; real_obstacle-1; i++)
  {
      p.y += obstacle_data.data[i].y;
  }
  p.y += 0.3*p.y+0.7*obstacle_data.data[real_obstacle-1].y;
  p.x = obstacle_data.data[real_obstacle-1].x;
  if (p.x<2.5)
  {
    p.x = p.x-0.4;
  }
  else p.x += 0.4;

  return p;
}


void mode2_act()
{
  switch(mode_2_num)
  {
    //move to point
    case 1:
    {
      geometry_msgs::Point p;
      p = meanob();
      choose_point(&p);
      mode_2_num = 2;
      break;
    }

    case 2:
    {
      float du = bet_dis_robot(&destination,&robot_data);
      get_des();
      if (task_done == true && du <0.02)
      {
        mode_2_num = 3;
      }
      break;

    }
    case 3:
    {
      rememberred(dummy2);
      wobblefun();
      if (task_done == true)
      {
        mode_2_num = 4;
        geometry_msgs::Point q;
        q = meanob();
        q.x+=obstacle_data.data[real_obstacle-1].x-robot_data.x;
        choose_point(&q);
        mode_2_num = 4;
      }
      break;
    }

    case 4:
    {
      float du1 = bet_dis_robot(&destination,&robot_data);
      get_des();
      if (task_done == true && du1<0.02)
      {
        mode_2_num = 5;
      }
      break;
    }
    case 5:
    {
        rememberred(dummy2);
        wobblefun();
        if (task_done == true)
        {
          if (real_red==5)
          {
            mode=4;
          }
          else mode=3;
        }
    }
  }
}

void mode3_act()
{
  int lack = 5-real_red;
  int k = lack;

  geometry_msgs::Point p;
  geometry_msgs::Point q;

  bool switchs = false;
  bool switchs2 = false;
  int i = 0;
  red_balls_data.data.clear();

  while(k >0)
  {
    lack = real_red;
    if (i == 0)
    {
      //spin to opposite
      if (task_done == true)
      {
        i=1;
        //task false

      }
    }
    if (i=1 && switchs == false && switchs2 == false)
    {
      wobblefun();
      rememberred(dummy2);
      if (task_done == true)
      {
        switchs == true;
      }
    }
    else if (switchs == true && switchs2 == false)
    {
      if (real_red != 0)
      {
        p.x = red_balls_data.data[lack-k].x;
        p.y = red_balls_data.data[lack-k].y;
      }
      else mode=4;

      //publish to red ball
      if (task_done == true)
      {
        switchs2 == true;
      }
    }
    else if( switchs == true && switchs2 == true)
    {
      p.x = green_ball_data.data[0].x;
      p.y = green_ball_data.data[0].y;
      //publish to green ball
      if (task_done == true)
      {
        // publsih task_done = false
        switchs2 = false;
        switchs = false;
        k--;
      }
    }
  }
}

void mode4_act()
{
  geometry_msgs::Point a;
  geometry_msgs::Point b;
  bool switchs = false;
  bool switchs2 = false;
  while( score < 6)
  {
    rememberred(dummy2);
    if (task_done == false && switchs ==false && switchs2 == false)
    {
      if (real_red ==0)
      {
        wobblefun();
        rememberred(dummy2);
      }
      else
      {
        a.x =red_balls_data.data[0].x;
        a.y= red_balls_data.data[0].y;
      }
      //message store also
      switchs = true;
    }

    else if (task_done== true && switchs == true && switchs2 == false)
    {
      a.x =green_ball_data.data[0].x;
      a.y =green_ball_data.data[0].y;
      if (bet_dis_robot(&a, &robot_data)<0.02)
      {
        switchs = false;
        switchs2 =true;
        // task false;
      }
    }
    else if (task_done ==true && switchs == false && switchs2 == true)
    {
      //waiting
      //false

    }
    else if (task_done == true && switchs == true && switchs2 == true)
    {
      switchs = false;
      switchs = false;
      score+=1;
      //task_done = false;
    }
  }

}

//--------------------call back function -----------------------------------
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

//green ball call back function only for current value = finished
void green_Callback(const localization::multi_position &gbposition)
{
  dummy1.data = gbposition.data;
  dummy1.num = gbposition.num;
    if (dummy1.num !=0)
    {
      green_detect=true;
      green_ball_data.data[0].x = dummy1.data[0].x;
      green_ball_data.data[0].y = dummy1.data[0].y;
    }
}

//red_ball call_back function only for current value = finished
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

//robot_position call back function(only current value) == finished
void robot_Callback(const localization::robot_position &robotposition)
{
  robot_data.x = robotposition.x;
  robot_data.y = robotposition.y;
  robot_data.angle = robotposition.angle;
}

//---------------------------------------------------------------------------

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

int main(int argc, char **argv)
{

    ros::init(argc, argv, "Path_plan_node");                                       //init ros nodd
    ros::NodeHandle nh;                                                              //create node handler
    ros::Subscriber sub_obstacle = nh.subscribe("map/data/obstacle", 1, obstacle_Callback); //create subscriber
    ros::Subscriber sub_red_ball = nh.subscribe("map/data/red_ball", 1, red_Callback); //create subscriber
    ros::Subscriber sub_green_ball = nh.subscribe("map/data/green_ball",1,green_Callback);
    ros::Subscriber sub_robot_geo = nh.subscribe("map/data/robot",1,robot_Callback);
    //ros::Subscriber sub_task = nh.subscribe("path_plan/mode",1,task_done);
    //subscribe node of autodriving

    //ros::Publisher pub_point = nh.advertise<path_plan::operation>("path_plan/data",1);
    //ros::Publisher pub_mode = nh.advertise<path_plan::mode>("path_plan/wheel_mode",1);

    red_balls_data.data.resize(6);
    red_balls_data.num=5;

    obstacle_data.data.resize(4);
    obstacle_data.num=4;

   green_ball_data.num=1;

   green_ball_data.data.resize(1);
   green_ball_data.data[0].x=0;
   green_ball_data.data[0].y=0;

    while(ros::ok)
    {
      //declare the topic messag
      pathplan();
      ros::spinOnce();
    }
    return 0;
}
