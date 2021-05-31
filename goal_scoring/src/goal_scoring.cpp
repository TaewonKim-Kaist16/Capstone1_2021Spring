#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "core_msgs/plane_info.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/calib3d.hpp"
#include <iostream>

using namespace cv;
using namespace std;

float intrinsic_data[9] = {589.3666835307066, 0.0, 319.5, 0.0, 589.3666835307066, 239.5, 0.0, 0.0, 1.0};
float distortion_data[5] = {0, 0, 0, 0, 0};
float near_plane = 0.006;
float far_plane = 15;
float curr_time, start_time;

const int max_value_H = 360 / 2;
const int max_value = 255;
int low_H = 0, low_S = 72, low_V = 52;
int high_H = max_value_H, high_S = max_value, high_V = max_value;
int low_H_red1 = 0, low_H_red2 = 120, low_H_green = 53;
int high_H_red1 = 11, high_H_red2 = max_value_H, high_H_green = 80;
float green_distance_threshold = 0.18;
float slow_threshold = 0.23;
float green_distance_after = 0.35;
float grip_wait_time = 2.8;
bool is_goal = false;
bool is_goal_temp = false;
bool after_goal = false;
bool after_goal_end = false;

Mat image_distort, image_brg, mask;
ros::Publisher pub;
ros::Publisher pub_cmd_vel;
ros::Publisher pub_grip_main;
ros::Publisher pub_is_goal;
ros::Publisher pub_grip_arm;

void green_mask()
{
    Mat frame_HSV;
    cvtColor(image_brg, frame_HSV, COLOR_BGR2HSV);
    inRange(frame_HSV, Scalar(low_H_green, low_S, low_V), Scalar(high_H_green, high_S, high_V), mask);
}

void goal_approach()
{
    Mat image;
    if (!image_distort.data)
    {
        printf("No image data\n");
        return;
    }
    Mat intrinsic = Mat(3, 3, CV_32F, intrinsic_data);
    Mat distCoeffs = Mat(1, 5, CV_32F, distortion_data);
    //undistort(image_distort, image, intrinsic, distCoeffs);
    image = image_distort;
    cv::imshow("Image", image);
    std_msgs::Bool msg;
    int rows, cols;
    rows = image.rows;
    cols = image.cols;
    int i, j;
    float green_distance = 100;
    geometry_msgs::Twist vel_msg;
    for (i = 0; i < rows; ++i)
    {
        for (j = 0; j < cols; ++j)
        {
            if (mask.at<uint8_t>(i, j) != 0 && green_distance > image.at<float>(i, j))
            {
                green_distance = image.at<float>(i, j);
            }
        }
    }
    printf("green distance: %f\n", green_distance);
    vel_msg.angular.z = 0;
    if (after_goal)
    {
        if (green_distance <= green_distance_after)
        {
            vel_msg.linear.x = -3;
            pub_cmd_vel.publish(vel_msg);
        }
        else
        {
            vel_msg.linear.x = 0;
            pub_cmd_vel.publish(vel_msg);
            geometry_msgs::Twist grip_down;
            grip_down.angular.x = 0;
            grip_down.angular.y = 0;
            grip_down.angular.z = 0;
            grip_down.linear.x = 0;
            grip_down.linear.y = 0;
            grip_down.linear.z = 0;
            pub_grip_main.publish(grip_down);
            //waitKey(15000);
            after_goal_end = true;
            start_time = curr_time;
	    geometry_msgs::Twist arm_wide;
            arm_wide.angular.x = 0;
            arm_wide.angular.y = 0;
            arm_wide.angular.z = 0;
            arm_wide.linear.x = 0;
            arm_wide.linear.y = 0;
            arm_wide.linear.z = 0;
            pub_grip_arm.publish(arm_wide);
        }
        return;
    }
    if (green_distance >= 100)
    {
        printf("green is not found! try again.\n");
        std_msgs::Bool is_goal_pub;
        is_goal_pub.data = false;
        msg.data = false;
        vel_msg.linear.x = 0;
        vel_msg.angular.z = 0;
        is_goal = false;
        pub_is_goal.publish(is_goal_pub);
        pub_cmd_vel.publish(vel_msg);
        pub.publish(msg);
        return;
    }
    if (green_distance <= green_distance_threshold)
    {
        vel_msg.linear.x = 0;
        msg.data = false;
        pub_cmd_vel.publish(vel_msg);
        geometry_msgs::Twist arm_wide;
        arm_wide.angular.x = 0;
        arm_wide.angular.y = 0;
        arm_wide.angular.z = 0;
        arm_wide.linear.x = 0;
        arm_wide.linear.y = -0.03;
        arm_wide.linear.z = 0;
        pub_grip_arm.publish(arm_wide);
        after_goal = true;
        waitKey(2000);
    }
    else
    {
        if (green_distance <= slow_threshold)
        {
            vel_msg.linear.x = 1.5;
        }
        else
        {
            vel_msg.linear.x = 3;
        }
        msg.data = false;
    }
    pub.publish(msg);
    pub_cmd_vel.publish(vel_msg);
    //cv::imshow("HaHaHa", image_brg);
    cv::waitKey(1);
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        image_brg = cv_bridge::toCvShare(msg, "bgr8")->image; //transfer the image data into buffer
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
    if (is_goal && !after_goal_end)
    {
        green_mask();
    }
}

void depthCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        cv_bridge::CvImageConstPtr depth_img_cv;
        depth_img_cv = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1); //transfer the image data into buffer
        depth_img_cv->image.convertTo(image_distort, CV_32F, 0.001);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to '16UC1'.", msg->encoding.c_str());
    }
    if (is_goal && !after_goal_end)
    {
        goal_approach();
    }
}

void goalCallback(const std_msgs::BoolConstPtr &msg)
{
    if (msg->data && (!is_goal) && (!is_goal_temp))
    {
        geometry_msgs::Twist grip_up;
        grip_up.angular.x = 0;
        grip_up.angular.y = 0;
        grip_up.angular.z = 0;
        grip_up.linear.x = 0;
        grip_up.linear.y = 0;
        grip_up.linear.z = 0.23;
        pub_grip_main.publish(grip_up);
        after_goal = false;
        after_goal_end = false;
        start_time = curr_time;
        is_goal_temp = true;
        //waitKey(15000);
    }
    else if (!is_goal_temp)
    {
        is_goal = msg->data;
    }
}

void timeCallback(const std_msgs::Float64ConstPtr &msg)
{
    curr_time = msg->data;
    if (is_goal && !after_goal && curr_time > 6 + start_time)
    {
        printf("goal_failed! try again.\n");
        geometry_msgs::Twist vel_msg;
        std_msgs::Bool succ_msg;
        std_msgs::Bool is_goal_pub;
        is_goal_pub.data = false;
        succ_msg.data = false;
        vel_msg.linear.x = 0;
        vel_msg.angular.z = 0;
        is_goal = false;
        pub_is_goal.publish(is_goal_pub);
        pub_cmd_vel.publish(vel_msg);
        pub.publish(succ_msg);
    }
    if (is_goal_temp && curr_time > grip_wait_time + start_time)
    {
        printf("grip up wait complete!\n");
        is_goal_temp = false;
        is_goal = true;
        start_time = curr_time;
    }
    if (after_goal_end && curr_time > grip_wait_time + start_time)
    {
        printf("grip down wait complete!\n");
        std_msgs::Bool is_goal_pub;
        std_msgs::Bool succ_msg;
        is_goal_pub.data = false;
        succ_msg.data = true;
        after_goal_end = false;
        is_goal = false;
        pub_is_goal.publish(is_goal_pub);
        pub.publish(succ_msg);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "goal_scoring_node");                                          //init ros node
    ros::NodeHandle nh;                                                                  //create node handler
    image_transport::ImageTransport it(nh);                                              //create image transport and connect it to node hanlder
    image_transport::Subscriber sub = it.subscribe("/kinect_rgb_top", 1, imageCallback); //create subscriber
    image_transport::Subscriber sub_depth = it.subscribe("/kinect_depth_top", 1, depthCallback);
    ros::Subscriber sub_is_goal = nh.subscribe<std_msgs::Bool>("/is_goal", 1, goalCallback);
    ros::Subscriber sub_time = nh.subscribe<std_msgs::Float64>("/simulTime", 1, timeCallback);

    pub = nh.advertise<std_msgs::Bool>("/goal_success", 10); //setting publisher
    pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    pub_grip_main = nh.advertise<geometry_msgs::Twist>("/GripperMain/cmd_vel", 10);
    pub_grip_arm = nh.advertise<geometry_msgs::Twist>("/GripperArm/cmd_vel", 10);
    pub_is_goal = nh.advertise<std_msgs::Bool>("/is_goal", 10);

    std_msgs::Bool is_goal_pub;
    is_goal_pub.data = false;
    pub_is_goal.publish(is_goal_pub);

    ros::spin(); //spin.
    return 0;
}
