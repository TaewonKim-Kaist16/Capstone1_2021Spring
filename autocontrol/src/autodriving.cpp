#include <ros/ros.h>
#include "string.h"
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <cmath>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <algorithm>
#include "localization/robot_position.h"
#include "autocontrol/motion.h"

using namespace std;


struct ABSOLUTE_POSITION {
    geometry_msgs::Point pose;
    float angle;
};

autocontrol::motion motion;
ABSOLUTE_POSITION robot_geometry;
geometry_msgs::Twist vel_msg;
std_msgs::String sys_cmd;
bool flag_motion_cb = false;
int process_num;

geometry_msgs::Point wobble_ref,wobble_p1, wobble_p2;

float dist_error, dist_error_pre, dist_error_dot, dist_error_tot;
float ang_error, ang_error_pre, ang_error_dot, ang_error_tot;

std_msgs::Float32 timestamp_start, timestamp_current;


float convert_ang(float angle, int option, bool use_radian=false) {
    /*  
        Convert angle into range 0~359
        option 1 Input : >:-90, ^:0, <:90, v:180(-180)
        option 2 Input : >:0, ^:90, <:180(-180), v:-90
        output : >:0, ^:90, <:180, v:270
        use_radian : true if input is radian, false if input is degree
        return is degree
    */

    float cvt_angle;

    if (use_radian) {
        angle = angle * 180 / M_PI;
    }

    if (option == 1) {
        if (angle >=-90 && angle <= 180) { cvt_angle = angle + 90; }
        else if (angle >= -180 && angle < -90) { cvt_angle = angle + 450; }
        else { cout << "check autodriving convert angle!!!!" << endl; }
    }

    else if (option == 2) {
        if (angle >= 0 && angle <=180) { cvt_angle = angle; }
        else if (angle >= -180 && angle < 0) { cvt_angle = angle + 360; }
        else { cout << "check autodriving convert angle!!!!" << endl; }
    }

    else { cout << "check autodriving convert angle!!!!" << endl; }

    return cvt_angle;

}


void arrange_angle(float x, float y) {

    float kp=4,kd=0.002,ki=0;
    ang_error_pre = ang_error;

    float x_diff = x - robot_geometry.pose.x;
    float y_diff = y - robot_geometry.pose.y;
    float angle = atan2(y_diff,x_diff);
    
    float rob_ang = convert_ang(robot_geometry.angle,1,true);
    float direction = convert_ang(angle,2,true);

    if (direction >= rob_ang) {
        if (direction - rob_ang <= 180) { ang_error = direction - rob_ang; }
        else { ang_error = direction - rob_ang - 360; }
    }
    else {
        if (direction - rob_ang >= -180) { ang_error = direction - rob_ang; }
        else { ang_error = direction - rob_ang + 360; }
    }

    // if (abs(direction - rob_ang) < abs(direction - (rob_ang + 360))) {
    //     ang_error = direction - rob_ang;
    // }
    // else {
    //     ang_error = direction - (rob_ang + 360);
    // }
    // cout << "ang_error : " << ang_error << endl;

    float dt = max(timestamp_current.data - timestamp_start.data,(float)0.01);
    ang_error = ang_error * M_PI / 180; //convert to radian
    ang_error_dot = (ang_error - ang_error_pre) / dt;
    ang_error_tot = ang_error * dt + ang_error_tot;

    vel_msg.linear.x = 0;
    float ang_vel = kp*ang_error+kd*ang_error_dot+ki*ang_error_tot;
    if (ang_vel >= 0) { vel_msg.angular.z = min(ang_vel, (float)4); }
    else { vel_msg.angular.z = max(ang_vel, (float)-4); }
    
    timestamp_start.data = timestamp_current.data;

    cout << "dt : " << dt << endl;
    cout << "error : " << ang_error << " | " << kp*ang_error << endl;
    cout << "error dot : " << ang_error_dot << " | " << kd*ang_error_dot << endl;
    cout << "error tot : " << ang_error_tot << " | " << ki*ang_error_tot << endl;
    cout << "ang vel : " << vel_msg.angular.z << endl;

}


void go_forward(float x, float y) {
    float kp=3,kd=0.02,ki=0.05;
    float kp_ang=2,kd_ang=0.005,ki_ang=0.05;
    dist_error_pre = dist_error;
    

    float x_diff = x - robot_geometry.pose.x;
    float y_diff = y - robot_geometry.pose.y;
    float distance = sqrt(pow(x_diff,2)+pow(y_diff,2));
    float angle = atan2(y_diff,x_diff);
    
    float rob_ang = convert_ang(robot_geometry.angle,1,true);
    float direction = convert_ang(angle,2,true);

    if (abs(direction - rob_ang) < abs(direction - (rob_ang + 360))) {
        ang_error = direction - rob_ang;
    }
    else {
        ang_error = direction - (rob_ang + 360);
    }

    float ang_vel = kp_ang*ang_error+kd_ang*ang_error_dot+ki_ang*ang_error_tot;
    if (ang_vel >= 0) { vel_msg.angular.z = min(ang_vel, (float)1); }
    else { vel_msg.angular.z = max(ang_vel, (float)-1); }

    float dt = max(timestamp_current.data - timestamp_start.data,(float)0.01);
    dist_error = distance;
    dist_error_dot = (dist_error - dist_error_pre) / dt;
    dist_error_tot = dist_error * dt + dist_error_tot;

    float lin_vel = kp*dist_error + kd*dist_error_dot + ki*dist_error_tot;
    vel_msg.linear.x = min(lin_vel,(float)4);

    timestamp_start.data = timestamp_current.data;

    cout << "dt : " << dt << endl;
    cout << "error : " << dist_error << " | " << kp*dist_error << endl;
    cout << "error dot : " << dist_error_dot << " | " << kd*dist_error_dot << endl;
    cout << "error tot : " << dist_error_tot << " | " << ki*dist_error_tot << endl;
    cout << "ang vel : " << vel_msg.angular.z << endl;
    cout << "lin vel : " << vel_msg.linear.x << endl;

}


void cal_wobbling_range() {
    float rob_ang = convert_ang(robot_geometry.angle,1,true);
    wobble_ref.x = robot_geometry.pose.x + 2*cos(rob_ang*M_PI/180);
    wobble_ref.y = robot_geometry.pose.y + 2*sin(rob_ang*M_PI/180);
    
    float th1 = (rob_ang + motion.angle) * M_PI / 180;
    wobble_p1.x = robot_geometry.pose.x + 2*cos(th1);
    wobble_p1.y = robot_geometry.pose.y + 2*sin(th1);

    float th2 = (rob_ang - motion.angle) * M_PI / 180;
    wobble_p2.x = robot_geometry.pose.x + 2*cos(th2);
    wobble_p2.y = robot_geometry.pose.y + 2*sin(th2);

    cout << "ref x : " << wobble_ref.x << " ref y : " << wobble_ref.y << endl
         << "wobble p1 x : " << wobble_p1.x << " wobble p1 y : " << wobble_p1.y << endl
         << "wobble p2 x : " << wobble_p2.x << " wobble p2 y : " << wobble_p2.y << endl;
}


void move_single_point(){

    if (flag_motion_cb == true) {
        cout << "process num : " << process_num << endl;
        if (process_num == 0) {
            arrange_angle(motion.x, motion.y);
            cout << "error : " << ang_error << " diff : " << abs(ang_error - ang_error_pre) << endl;
            if (abs(ang_error) < 2*M_PI/180 && abs(ang_error - ang_error_pre) < 1*M_PI/180 ) {
                ang_error=0, ang_error_pre=0, ang_error_dot=0, ang_error_tot=0;
                process_num = 1;
            }
        }
        else if (process_num == 1) {
            go_forward(motion.x, motion.y);
            if (dist_error < 0.05) {
                process_num = 2;
            }
        }
        else {
            flag_motion_cb = false;
            dist_error=0, dist_error_pre=0, dist_error_dot=0, dist_error_tot=0;
            ang_error=0, ang_error_pre=0, ang_error_dot=0, ang_error_tot=0;
        }
    }
    else {
        vel_msg.linear.x = 0;
        vel_msg.angular.z = 0;
        process_num = 0;
    }
}


void wobbling() {
    if (flag_motion_cb == true) {
        cout << "process num : " << process_num << endl;
        if (process_num == 0) {
            arrange_angle(wobble_p1.x,wobble_p1.y);
            if (abs(ang_error) < 4*M_PI/180 && abs(ang_error - ang_error_pre) < 3*M_PI/180 ) {
                ang_error=0, ang_error_pre=0, ang_error_dot=0, ang_error_tot=0;
                process_num = 1;
            }
        }
        else if (process_num == 1) {
            arrange_angle(wobble_p2.x,wobble_p2.y);
            if (abs(ang_error) < 4*M_PI/180 && abs(ang_error - ang_error_pre) < 3*M_PI/180 ) {
                ang_error=0, ang_error_pre=0, ang_error_dot=0, ang_error_tot=0;
                process_num = 2;
            }            
        }
        else if (process_num == 2) {
            arrange_angle(wobble_ref.x,wobble_ref.y);
            if (abs(ang_error) < 2*M_PI/180 && abs(ang_error - ang_error_pre) < 1*M_PI/180 ) {
                ang_error=0, ang_error_pre=0, ang_error_dot=0, ang_error_tot=0;
                process_num = 3;
            }
        }
        else {
            flag_motion_cb = false;
            dist_error=0, dist_error_pre=0, dist_error_dot=0, dist_error_tot=0;
            ang_error=0, ang_error_pre=0, ang_error_dot=0, ang_error_tot=0;            
        }

    }
    else {
        vel_msg.linear.x = 0;
        vel_msg.angular.z = 0;
        process_num = 0;
    }
}

void init_vel() {
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;
}


void sys_cb(std_msgs::String msg) {
    sys_cmd.data = msg.data;
    if (msg.data == "autodriving") {
        if (motion.msg == "driving") {
            move_single_point();
        }
        else if (motion.msg == "wobbling") {
            wobbling();
        }
        else if (motion.msg == "rotation") {
            arrange_angle(motion.x,motion.y);
        }
    }
}



void motion_cb(autocontrol::motion m) {
    flag_motion_cb = true;
    process_num = 0;
    motion.x = m.x;
    motion.y = m.y;
    motion.angle = m.angle;
    motion.msg = m.msg;
    dist_error=0, dist_error_pre=0, dist_error_dot=0, dist_error_tot=0;
    ang_error=0, ang_error_pre=0, ang_error_dot=0, ang_error_tot=0;
    // cout << "path point x, y : " << motion.x << " " << motion.y << endl;

    if (motion.msg == "wobbling") {
        cal_wobbling_range();
    }
}

void robot_cb(localization::robot_position robot_pose) {
    robot_geometry.pose.x = robot_pose.x;
    robot_geometry.pose.y = robot_pose.y;
    robot_geometry.angle = robot_pose.angle;
}

void time_cb(std_msgs::Float32 time) {
    timestamp_current = time;
    // cout << "time : " << timestamp_current << endl;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "autodriving");
    ros::NodeHandle nh;

    ros::Subscriber sub_time = nh.subscribe("/simTime",1,time_cb);

    ros::Subscriber sub_sys_cmd = nh.subscribe("/sys_cmd",1000,sys_cb);
    ros::Subscriber sub_path = nh.subscribe("/motion", 1, motion_cb);
    ros::Subscriber sub_robot = nh.subscribe("/map/data/robot",1,robot_cb);

    ros::Publisher pub_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);

    init_vel();

    ros::Rate loop_rate(5);
    while(ros::ok()){
        ros::spinOnce();

        if (sys_cmd.data != "wait") {
            pub_vel.publish(vel_msg);
        }

        loop_rate.sleep();
    }
    return 0;
}