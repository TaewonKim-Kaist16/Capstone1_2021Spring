#include <ros/ros.h>
#include "string.h"
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <cmath>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <algorithm>
#include "localization/robot_position.h"
#include "autocontrol/motion.h"
#include "localization/multi_position.h"


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
geometry_msgs::Point harvesting_p1;

localization::multi_position red_balls_data;
bool is_align = false;
bool ball_approach_sucess = false;

float dist_error, dist_error_pre, dist_error_dot, dist_error_tot;
float ang_error, ang_error_pre, ang_error_dot, ang_error_tot;

float timestamp_start, timestamp_current;

ros::Publisher pub_is_align;

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

    float dt = max(timestamp_current - timestamp_start,(float)0.01);
    ang_error = ang_error * M_PI / 180; //convert to radian
    ang_error_dot = (ang_error - ang_error_pre) / dt;
    ang_error_tot = ang_error * dt + ang_error_tot;

    vel_msg.linear.x = 0;
    float ang_vel = kp*ang_error+kd*ang_error_dot+ki*ang_error_tot;
    if (ang_vel >= 0) { vel_msg.angular.z = min(ang_vel, (float)4); }
    else { vel_msg.angular.z = max(ang_vel, (float)-4); }

    timestamp_start = timestamp_current;

    cout << "dt : " << dt << endl;
    cout << "error : " << ang_error << " | " << kp*ang_error << endl;
    cout << "error dot : " << ang_error_dot << " | " << kd*ang_error_dot << endl;
    cout << "error tot : " << ang_error_tot << " | " << ki*ang_error_tot << endl;
    cout << "ang vel : " << vel_msg.angular.z << endl;

}


void go_forward(float x, float y,float max_lin=4.0,float max_ang=1.5) {
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
    if (ang_vel >= 0) { vel_msg.angular.z = min(ang_vel, max_ang); }
    else { vel_msg.angular.z = max(ang_vel, -max_ang); }

    float dt = max(timestamp_current - timestamp_start,(float)0.01);
    dist_error = distance;
    dist_error_dot = (dist_error - dist_error_pre) / dt;
    dist_error_tot = dist_error * dt + dist_error_tot;

    float lin_vel = kp*dist_error + kd*dist_error_dot + ki*dist_error_tot;
    vel_msg.linear.x = min(lin_vel,max_lin);

    timestamp_start = timestamp_current;

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


void cal_harvesting_point() {
    float margin = 0.3;

    float x_diff = motion.x - robot_geometry.pose.x;
    float y_diff = motion.y - robot_geometry.pose.y;
    float distance = sqrt(pow(x_diff,2)+pow(y_diff,2));

    harvesting_p1.x = motion.x - x_diff * (margin/distance);
    harvesting_p1.y = motion.y - y_diff * (margin/distance);
    cout << "harvesting x, y : " << harvesting_p1.x << " " << harvesting_p1.y << endl;
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
    if (flag_motion_cb == true && ball_approach_sucess == false) {
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
        ball_approach_sucess = false;
        vel_msg.linear.x = 0;
        vel_msg.angular.z = 0;
        process_num = 0;
    }
}


void harvesting(){
    if (flag_motion_cb == true) {
        cout << "process num : " << process_num << endl;
        float x, y;

        if (process_num == 0) {
            float dist = sqrt(pow(motion.x-robot_geometry.pose.x,2)+pow(motion.y-robot_geometry.pose.y,2));
            if (dist < 0.04) {
                process_num = 2;
            }
            else {
                arrange_angle(harvesting_p1.x, harvesting_p1.y);
                cout << "error : " << ang_error << " diff : " << abs(ang_error - ang_error_pre) << endl;
                if (abs(ang_error) < 2*M_PI/180 && abs(ang_error - ang_error_pre) < 1*M_PI/180 ) {
                    ang_error=0, ang_error_pre=0, ang_error_dot=0, ang_error_tot=0;
                    process_num = 1;
                }
            }
        }
        else if (process_num == 1) {
            go_forward(harvesting_p1.x, harvesting_p1.y);
            if (dist_error < 0.04) {
                ang_error=0, ang_error_pre=0, ang_error_dot=0, ang_error_tot=0;
                process_num = 2;
            }
        }
        else if (process_num == 2) {
            float min_dist = 1000;
            for (int i=0; i<red_balls_data.num; i++) {
                float dist = sqrt(pow(red_balls_data.data[i].x-robot_geometry.pose.x,2)+pow(red_balls_data.data[i].y-robot_geometry.pose.y,2));
                if (dist < min_dist) {
                    x = red_balls_data.data[i].x;
                    y = red_balls_data.data[i].y;
                    min_dist = dist;
                }
            }

            arrange_angle(x,y);
            cout << "error : " << ang_error << " diff : " << abs(ang_error - ang_error_pre) << endl;
            if (abs(ang_error) < 0.5*M_PI/180 && abs(ang_error - ang_error_pre) < 1*M_PI/180 ) {
                ang_error=0, ang_error_pre=0, ang_error_dot=0, ang_error_tot=0;
                process_num = 3;
                is_align = true;
            }
        }
        // else if (process_num == 3) {
        //     go_forward(x,y,1,0);
        //     if (dist_error < 0.1) {
        //         process_num = 4;
        //         is_align = true;
        //     }
        // }
        else {
            std_msgs::Bool is_align_pub;
            is_align_pub.data = true;
            pub_is_align.publish(is_align_pub);

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
            // cout << "move to point " << motion.x << ", " << motion.y << endl;
            move_single_point();
        }
        else if (motion.msg == "wobbling") {
            // cout << "wobbling wit angle " << motion.angle << endl;
            wobbling();
        }
        else if (motion.msg == "rotation") {
            // cout << "rotate heading to point " << motion.x << ", " << motion.y << endl;
            arrange_angle(motion.x,motion.y);
        }
        else if (motion.msg == "harvesting") {
            // cout << "harvesting motion at point " << motion.x << ", " << motion.y << endl;
            harvesting();
        }
        else if (motion.msg == "") {}
        else {
            cout << "Invalid motion command" << endl;
            flag_motion_cb = false;
        }
    }
}


void motion_cb(autocontrol::motion msg) {
    flag_motion_cb = true;
    process_num = 0;
    motion.x = msg.x;
    motion.y = msg.y;
    motion.angle = msg.angle;
    motion.msg = msg.msg;
    dist_error=0, dist_error_pre=0, dist_error_dot=0, dist_error_tot=0;
    ang_error=0, ang_error_pre=0, ang_error_dot=0, ang_error_tot=0;
    // cout << "path point x, y : " << motion.x << " " << motion.y << endl;

    if (motion.msg == "wobbling") {
        cal_wobbling_range();
    }
    else if (motion.msg == "harvesting") {
        cal_harvesting_point();
    }
}

void red_ball_cb(localization::multi_position msg) {
    red_balls_data = msg;
}

void robot_cb(localization::robot_position msg) {
    robot_geometry.pose.x = msg.x;
    robot_geometry.pose.y = msg.y;
    robot_geometry.angle = msg.angle;
}

void time_cb(const std_msgs::Float64ConstPtr &msg){
    timestamp_current = msg->data;
    // cout << "time : " << timestamp_current << endl;
}

void align_cb(std_msgs::Bool msg) {
    is_align = msg.data;
    if (is_align == false) {
        flag_motion_cb = false;
    }
}

void ball_approach_success_cb(std_msgs::Bool msg) {
    ball_approach_sucess = msg.data;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "autodriving");
    ros::NodeHandle nh;

    ros::Subscriber sub_time = nh.subscribe<std_msgs::Float64>("/simulTime", 1, time_cb);

    ros::Subscriber sub_sys_cmd = nh.subscribe("/sys_cmd",1000,sys_cb);
    ros::Subscriber sub_path = nh.subscribe("/motion", 1, motion_cb);
    ros::Subscriber sub_robot = nh.subscribe("/map/data/robot",1,robot_cb);
    ros::Subscriber sub_rb = nh.subscribe("/map/data/red_ball",1,red_ball_cb);
    ros::Subscriber sub_is_align = nh.subscribe("/is_align", 1, align_cb);
    ros::Subscriber sub_ball_approach = nh.subscribe("/ball_approach_success",1,ball_approach_success_cb);

    ros::Publisher pub_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    pub_is_align = nh.advertise<std_msgs::Bool>("/is_align", 10);

    init_vel();

    ros::Rate loop_rate(5);
    while(ros::ok()){
        ros::spinOnce();

        if (sys_cmd.data == "wait") {
            // cout << "wait mode" << endl;
        }
        else if (is_align == true) {
            cout << "ball_approach activate" << endl;
        }
        else {
            pub_vel.publish(vel_msg);
        }

        loop_rate.sleep();
    }
    return 0;
}
