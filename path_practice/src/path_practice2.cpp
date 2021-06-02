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
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include "autocontrol/motion.h"

#include <iostream>

using namespace std;

localization::multi_position obstacle_data;
localization::multi_position red_store;
localization::multi_position red_current;
localization::multi_position green_current;
localization::robot_position robot_data;
localization::multi_position inline_object;

ros::Publisher pub_motion;


int mode =1;
std_msgs::String sys_cmd;
bool task_done;
int process_num=0;
int sub_path_num=0;

struct MIN_DIST {
  float dist;
  float x;
  float y;
};

struct OBJECT_INFOR
{
  vector<int> order;
  vector<float> x_intercept;
  vector<float> y_intercept;
  vector<float> distance;
  int size;
  vector<geometry_msgs::Point> odometry;
  
};

MIN_DIST red_min_dist;

OBJECT_INFOR in_path_object_infor;

//---------------declare functions ----------------------------------------
void mode1_act();

void save_min_red()
{
  cout <<"enter the update red function"<<endl;
  for (int i=0;i<6;i++) {
    float diff_x = red_current.data[i].x-robot_data.x;
    float diff_y = red_current.data[i].y-robot_data.y;
    float dist = sqrt(pow(diff_x,2)+pow(diff_y,2));
    for(int i = 0; i<red_current.num; i++)
    {
      if(red_current.data[i].x !=50 && red_current.data[i].x !=100)
      {
        if (red_min_dist.dist > dist) {
          red_min_dist.x = red_current.data[i].x;
          red_min_dist.y = red_current.data[i].y;
          red_min_dist.dist = dist;
        }
      }  
    }

  }
}

vector<int> detect_inline_object(float x, float y)
{

  inline_object.data.clear();
  inline_object.num = 0;
  in_path_object_infor.order.clear();
  in_path_object_infor.x_intercept.clear();
  in_path_object_infor.y_intercept.clear();
  in_path_object_infor.odometry.clear();
  in_path_object_infor.size =0;
  vector<int> line_inf(2,0);

  float m = (y-robot_data.y)/(x-robot_data.x);

  float b = robot_data.y-m*robot_data.x;

  float vertical_distance;

  for (int i=0; i<obstacle_data.num; i++)
  {
    vertical_distance = abs(m*obstacle_data.data[i].x-obstacle_data.data[i].y+b)/sqrt(m*m+1);
    if (vertical_distance <= 0.3)
    {
      inline_object.data.push_back(obstacle_data.data[i]);
      inline_object.num+=1;

      in_path_object_infor.size += 1;
      int size_recent = in_path_object_infor.size-1;
      in_path_object_infor.odometry.resize(size_recent);
      in_path_object_infor.order.resize(size_recent);
      in_path_object_infor.x_intercept.resize(size_recent);
      in_path_object_infor.y_intercept.resize(size_recent);
      in_path_object_infor.odometry.push_back(obstacle_data.data[i]);
      in_path_object_infor.x_intercept.push_back(obstacle_data.data[i].x);
      in_path_object_infor.y_intercept.push_back(obstacle_data.data[i].y);
      in_path_object_infor.order.push_back(size_recent);
      cout <<"object order size " <<in_path_object_infor.order.size()<<" size"<<in_path_object_infor.size<<endl;
    }
  }

  for (int i = 0; i<red_current.num; i++)
  {
        vertical_distance = abs(m*red_current.data[i].x-red_current.data[i].y+b)/sqrt(m*m+1);
    if (vertical_distance <= 0.3)
    {
      inline_object.data.push_back(red_current.data[i]);
      inline_object.num+=1;

      in_path_object_infor.size += 1;
      int size_recent = in_path_object_infor.size-1;
      in_path_object_infor.odometry.resize(size_recent);
      in_path_object_infor.order.resize(size_recent);
      in_path_object_infor.x_intercept.resize(size_recent);
      in_path_object_infor.y_intercept.resize(size_recent);
      in_path_object_infor.odometry.push_back(red_current.data[i]);
      in_path_object_infor.x_intercept.push_back(red_current.data[i].x);
      in_path_object_infor.y_intercept.push_back(red_current.data[i].y);
      in_path_object_infor.order.push_back(size_recent);
    }
  }

  for (int i =0; i<inline_object.num; i++)
  {
    cout<<"the inline object :"<<i<<" th : "<<inline_object.data[i].x<<"   "<<inline_object.data[i].y<<endl;
  }
  line_inf[0]=m;
  line_inf[1]=b;
  for (int i=0; i<in_path_object_infor.size; i++)
  {
    cout<<"in path structure : "<< in_path_object_infor.odometry[i].x<<endl;
  }
  cout <<"size of x_inteception number"<<in_path_object_infor.x_intercept.size()<<endl;
  cout <<"return the line value"<<endl;
  return line_inf;
}

void calculate_object_distance()
{
  float x_diff;
  float y_diff;
  in_path_object_infor.distance.clear();
  in_path_object_infor.distance.resize(in_path_object_infor.size);
  for (int i = 0; i<in_path_object_infor.size; i++)
  {
    x_diff = in_path_object_infor.x_intercept[i] - robot_data.x;
    y_diff = in_path_object_infor.y_intercept[i] - robot_data.y;
    in_path_object_infor.distance[i] = sqrt(pow(x_diff,2)+pow(y_diff,2));
  } 
}

void calculate_intercept(float *m, float* b)
{
  float tilt = *m;
  float origin_b = *b;
  cout << "inpath object infor.order size  "<<in_path_object_infor.order.size()<<endl;
  cout << "inpath object infor.size : "<<in_path_object_infor.size<<endl;
  
  for(int i = 0; i<in_path_object_infor.size; i++)
  {//calculate interception point on line
    in_path_object_infor.x_intercept[i] = (in_path_object_infor.odometry[i].x+tilt*in_path_object_infor.odometry[i].y-origin_b)/(tilt*tilt+1);
    in_path_object_infor.y_intercept[i] = tilt*in_path_object_infor.x_intercept[i] + origin_b;
  }
}

// void update()
// {
//   int size;
//   vector<int> index_store;
//
//   for (int i=0; i<red_current.num; i++)
//   {
//     for(int j=0; j<red_store.num; j++)
//     {
//       if((red_current.data[i].x-red_store.data[i].x)<0.1 && (red_current.data[i].y-red_store.data[i].y)<0.1 )
//       {
//         continue;
//         red_store.data[i].x
//       }
//     }
//     if(red_current.data[i].x != 50 || red_current.data[i].x != 100)
//     {
//       red_store.data.push_back(red_current.data[i]);
//       size = red_store.data.size();
//       red_store.num = size;
//     }
//   }
//   cout<<"red_balls num : "<<red_store.num<<endl;
//   for(int i=0; i<red_store.num; i++)
//   {
//     cout<<"red ball "<<i<<"th : "<<red_store.data[i].x<<"  "<<red_store.data[i].y<<endl;
//   }
// }
// int updating(localization::multi_position *recentone,localization::multi_position *oldone)
// {
//   geometry_msgs::Point p;
//   int a=0;
//   int rem=0;
//
//   //green_ball same data exist
//   bool b=false;
//
//   int c=0;
//   int d=0;
//   int count=0;
//   localization::multi_position rec=*recentone;
//   localization::multi_position old=*oldone;
//   localization::multi_position new1;
//   new1.data.clear();
//   new1.num=old.num-1;
//   rearrange(&rec);
//
//   //green ball detect confirm mode
//   if(green_detect==true)
//   {
//     for (int i=0; i<old.num; i++)
//     {
//       p.x=abs(green_ball_data.data[i].x-rec.data[i].x);
//       p.y=abs(green_ball_data.data[i].y-rec.data[i].y);
//       //if green ball does not exist at final position
//       if ((p.x<0.2 && p.y<0.2) && (old.num-1 !=i))
//       {
//         //have to change order
//         b=true;
//         //remember the index
//         rem=i;
//         c=1;
//       }
//       //for dummy data which filled at no real data exist
//       if (rec.data[i].x==50 || rec.data[i].x==100)
//       {
//         //count the number
//         d+=1;
//       }
//     }
//   }
//   //green ball reach its proper position
//   if(b==true)
//   {
//     p=rec.data[old.num-1];
//     rec.data[rem]=rec.data[old.num-1];
//     rec.data[old.num-1]=p;
//
//     // rearrange the order except dummy value
//     for(int j=rem; j<old.num-d-2; j++)
//     {
//       change_order(&rec.data[j],&rec.data[j+1]);
//     }
//   }
//   //up to now, green ball on the last position
//
//   //update the data
//   for(int i=0; i<rec.num-c; i++)
//   {
//     for(int j=0; j<new1.num; j++)
//     {
//       p.x=abs(old.data[j].x-rec.data[i].x);
//       p.y=abs(old.data[j].y-rec.data[i].y);
//
//       //if data same, update with recent data
//       if(p.x<0.2 && p.y<0.2)
//       {
//           new1.data.push_back(rec.data[i]);
//
//         //initalize
//         rec.data[i].x=0;
//         rec.data[i].y=0;
//         count+=1;
//       }
//     }
//   }
//   //fill other recent value
//   for(int i=0; i<rec.num-c; i++)
//   {
//     if((rec.data[i].x !=0) || (rec.data[i].x != 50) || (rec.data[i].x != 100))
//     {
//       new1.data.push_back(rec.data[i]);
//       count+=1;
//     }
//   }
//
//   rearrange(&new1);
//   new1.num=old.num;
//   new1.data.resize(old.num);
//
//   geometry_msgs::Point u;
//   u.x=1000; u.y=1000;
//
//   for (int i=count; i<old.num; i++)
//   {
//     new1.data.push_back(p);
//     new1.data.push_back(p);
//   }
//
//   *normal = old;
//   return count;
// }
void rearrange_inpath_object(float m, float b)
{
  float tilt = m;
  float translate = b;
  cout << "before calculating interception"<<endl;
  calculate_intercept(& tilt, & translate);
  cout <<" end intercetion calculation and ready to calculate object distance"<<endl;
  calculate_object_distance();
  cout << " calculate the object distance well"<<endl;
  geometry_msgs::Point swaping_dummy;
  float dummy_interception;

  for (int i=0; i<in_path_object_infor.size-1; i++)
  {
    for (int j=i+1; j<in_path_object_infor.size; j++)
    {
      if (in_path_object_infor.distance[i] > in_path_object_infor.distance[j])
      {
        swaping_dummy = in_path_object_infor.odometry[i];
        in_path_object_infor.odometry[i] = in_path_object_infor.odometry[j];
        in_path_object_infor.odometry[j] = swaping_dummy;

        dummy_interception = in_path_object_infor.x_intercept[i];
        in_path_object_infor.x_intercept[i] = in_path_object_infor.x_intercept[j];
        in_path_object_infor.x_intercept[j] = dummy_interception;
        dummy_interception = in_path_object_infor.y_intercept[i];
        in_path_object_infor.y_intercept[i] = in_path_object_infor.y_intercept[j];
        in_path_object_infor.y_intercept[j] = dummy_interception;
      }
    }
  }
}

void rotation(float x, float y)
{
  // cout<<"rotate activated : "<<x<<"      "<<y<<endl;
  autocontrol::motion motion;

  motion.msg ="rotation";
  motion.x =x;
  motion.y=y;
  pub_motion.publish(motion);

}


void wobble(float angle)
{
  autocontrol::motion motion;
  motion.msg = "wobbling";
  motion.angle = angle;

  save_min_red();
  //cout << "min red ball : "  << red_min_dist.x << " " << red_min_dist.y << "  dist : " << red_min_dist.dist << endl;
  pub_motion.publish(motion);
  // for (int i = 0; i<red_store.num; i++)
  // {
  //   cout<<"red_ball stored : "<<red_store<<endl;
  //   cout<<"red balls " <<i<<":   "<<red_store.data[i].x<<"   "<<red_store.data[i].y<<endl;
  // }
}

void calculate_sub_path()
{
  
}


void mode1_act()
{
  
  if(process_num ==0)
  { 

    rotation(1,2.5);

    if (task_done == true)
    {
      process_num=1;
      task_done =false;
    }
  }
  else if (process_num ==1)
  {
    wobble(25);
    // update();
    cout << "process " << 2 <<" started" << endl;

    if (task_done == true)
    {
      process_num = 2;
      task_done = false;
    }
  }
  else if (process_num ==2)
  {
    autocontrol::motion motion;
    motion.msg ="harvesting";
    motion.x =red_min_dist.x;
    motion.y=red_min_dist.y;
    pub_motion.publish(motion);
    if(task_done == true)
    {
      process_num=3;
      task_done = false;
    }

  //  cout << "wobbling is finished"<<endl;
  }
  else if (process_num == 3)
  {

    rotation(1.5,5);
    float x=1.5;
    float y= 5;
    if (task_done == true)
    { vector<int> line_information;
      line_information = detect_inline_object(x,y);
      cout <<"return well and wati the reaarange fucntion"<<endl;
      rearrange_inpath_object(line_information[0],line_information[1]);
      cout<<"rearrange is well doned"<<endl;
      for (int i =0; i<5; i++)
      {
        for(int j =0; j< in_path_object_infor.size; j++){
          cout << "in_line object : (shortest order)"<<in_path_object_infor.odometry[j].x<<" "<<in_path_object_infor.odometry[j].y<<endl;
        }
      }
      process_num=4;
      task_done =false;
    }
  }
  else if (process_num ==4){
    calculate_sub_path();
    



  }
}



void pathPlan()
{
 switch (mode)
 {
  case 1:
  {
    mode1_act();

  }
 }
}







//------------------------------call_back -------------------------------------

void obstacle_Callback(const localization::multi_position &obposition)
{
  obstacle_data.num = obposition.num;
  obstacle_data.data = obposition.data;


}

void red_Callback(const localization::multi_position &rbposition)
{
  red_current.data = rbposition.data;
  red_current.num = rbposition.num;

  if (red_current.num != 6)
  {
    //final index for case dectecting green ball
    red_current.data.resize(6);
    for (int i=5; i>red_current.num-1; i--)
    {
      red_current.data[i].x=50;
      red_current.data[i].y=50;
    }
  }
}

void green_Callback(const localization::multi_position &gbposition)
{

  green_current.data = gbposition.data;
  green_current.num = gbposition.num;
  //cout << " green_current" << green_current.data[0].x <<endl;
    // if (green_current.num !=0)
    // {
    //   green_ball_data.data[0].x = green_current.data[0].x;
    //   green_ball_data.data[0].y = green_current.data[0].y;
    // }
}

void robot_Callback(const localization::robot_position &robotposition)
{
  robot_data.x = robotposition.x;
  robot_data.y = robotposition.y;
  robot_data.angle = robotposition.angle;
}



void sys_cb(std_msgs::String msg) {
    sys_cmd.data = msg.data;
}

void task_done_Callback(const std_msgs::BoolConstPtr &msg) {
  task_done = msg->data;
  cout<<task_done<<endl;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "Path_practice2");                                       //init ros nodd
  ros::NodeHandle nh;

  ros::Subscriber sub_obstacle = nh.subscribe("map/data/obstacle", 1, obstacle_Callback); //create subscriber
  ros::Subscriber sub_red_ball = nh.subscribe("map/data/red_ball", 1, red_Callback); //create subscriber
  ros::Subscriber sub_green_ball = nh.subscribe("map/data/green_ball",1,green_Callback);
  ros::Subscriber sub_robot_geo = nh.subscribe("map/data/robot",1,robot_Callback);
//  ros::Subscriber sub_task = nh.subscribe("path_plan/mode",1,task_done_Callback);
   ros::Subscriber sub_sys_cmd = nh.subscribe("/sys_cmd",1000,sys_cb);
   ros::Subscriber sub_motion = nh.subscribe("/path_plan/mode",1,task_done_Callback);



  pub_motion = nh.advertise<autocontrol::motion>("motion",1);

  red_store.num = 0;
  red_store.data.clear();

  red_min_dist.dist = 1000;

  in_path_object_infor.order.clear();
  in_path_object_infor.x_intercept.clear();
  in_path_object_infor.y_intercept.clear();
  in_path_object_infor.odometry.clear();
  in_path_object_infor.size = 0;

  while(ros::ok())
  {
    if(sys_cmd.data != "wait" && sys_cmd.data != "")
    {

      pathPlan();
    }

    ros::spinOnce();
  }
  cout<< "while loop escaped"<<endl;
  return 0;
}
