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

using namespace std;


struct ABSOLUTE_POSITION {
    geometry_msgs::Point pose;
    float angle;
};

geometry_msgs::Point path_point;
ABSOLUTE_POSITION robot_geometry;
geometry_msgs::Twist vel_msg;
std_msgs::String sys_cmd;
bool flag_path_cb = false;
int process_num;

// float dist_error, dist_error_pre, dist_error_dot;
// float ang_error, ang_error_pre, ang_error_dot;

std_msgs::Float32 timestamp_start, timestamp_current;


void move_single_point(){

    if (flag_path_cb == true) {

        float x_diff = path_point.x - robot_geometry.pose.x;
        float y_diff = path_point.y - robot_geometry.pose.y;
        float angle = atan2(y_diff,x_diff);
        float distance = sqrt(pow(x_diff,2)+pow(y_diff,2));

        // Since the verticle set zero degree for robot, direction need to convert
        float direction;
        if (angle >= -M_PI/2 && angle < M_PI) { direction = angle - M_PI/2; }
        else { direction = angle + M_PI * 3/2; }

        float diff_angle = direction - robot_geometry.angle;
        cout << "distance : " << distance << endl << "diff angle : " << diff_angle << endl;
        cout << "porcess num : " << process_num << endl;
        if (process_num == 0) {
            if (abs(diff_angle) > 0.09) { // tolerance : about +-5 degree
                vel_msg.linear.y = 0;
                if (diff_angle > 0 && abs(diff_angle) <= M_PI) { vel_msg.angular.z = 0.15; }
                else if (diff_angle > 0 && abs(diff_angle) > M_PI) { vel_msg.angular.z = -0.15; }
                else if (diff_angle < 0 && abs(diff_angle) <= M_PI) { vel_msg.angular.z = -0.15; }
                else if (diff_angle < 0 && abs(diff_angle) > M_PI) { vel_msg.angular.z = 0.15; }
                else { vel_msg.angular.z = 0; }
            }
            else { process_num = 1; }
        }
        else if (process_num == 1) {
            vel_msg.linear.y = min(distance*(float)0.5,(float)0.1);

            if (diff_angle > 0) { vel_msg.angular.z = min(abs(diff_angle),(float)0.1); }
            else if (diff_angle < 0) { vel_msg.angular.z = -min(abs(diff_angle),(float)0.1); }
            else { vel_msg.angular.z = 0; }

            if (distance < 0.5) { process_num = 2; }
        }
        else if (process_num == 2) {
            vel_msg.linear.y = distance*(float)0.2;

            if (distance < 0.2) { process_num = 3; }
        }
        else {
            vel_msg.linear.y = 0;
            vel_msg.angular.z = 0;
            flag_path_cb = false;
        }

    }
    else {
        vel_msg.linear.y = 0;
        vel_msg.angular.z = 0;
    }
}

void init_vel() {
    vel_msg.linear.y = 0;
    vel_msg.angular.z = 0;
}


void sys_cb(std_msgs::String msg) {
    sys_cmd.data = msg.data;
    if (msg.data == "autodriving") {
        move_single_point();
    }
}

void path_cb(geometry_msgs::Point path_p) {
    flag_path_cb = true;
    process_num = 0;
    path_point.x = path_p.x;
    path_point.y = path_p.y;
    // cout << "path point x, y : " << path_point.x << " " << path_point.y << endl;
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

    ros::Subscriber sub_sys_cmd = nh.subscribe("/sys_cmd",1,sys_cb);
    ros::Subscriber sub_path = nh.subscribe("/path_point", 1, path_cb);
    ros::Subscriber sub_robot = nh.subscribe("/map/data/robot",1,robot_cb);

    ros::Publisher pub_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);

    init_vel();

    ros::Rate loop_rate(5);
    while(ros::ok()){
        ros::spinOnce();

        if (sys_cmd.data == "autodriving") {
            pub_vel.publish(vel_msg);
        }
    }
}