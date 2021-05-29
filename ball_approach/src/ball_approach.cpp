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
int red_cnt_threshold = 145000;
int slow_threshold = 110000;
bool is_align = true;

Mat image_distort, image_brg, mask;
ros::Publisher pub;
ros::Publisher pub_cmd_vel;
ros::Publisher pub_grip_main;
ros::Publisher pub_is_align;
ros::Publisher pub_grip_arm;

void ball_approach()
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
    image = image_brg;
    std_msgs::Bool msg;
    Mat frame_HSV, frame_red1, frame_red2, frame_threshold;
    cvtColor(image, frame_HSV, COLOR_BGR2HSV);
    inRange(frame_HSV, Scalar(low_H_red1, low_S, low_V), Scalar(high_H_red1, high_S, high_V), frame_red1);
    inRange(frame_HSV, Scalar(low_H_red2, low_S, low_V), Scalar(high_H_red2, high_S, high_V), frame_red2);
    frame_threshold = frame_red1 | frame_red2;
    int rows, cols;
    rows = frame_threshold.rows;
    cols = frame_threshold.cols;
    int i, j;
    int red_cnt = 0;
    geometry_msgs::Twist vel_msg;
    for (i = 0; i < rows; ++i)
    {
        for (j = 0; j < cols; ++j)
        {
            if (frame_threshold.at<uint8_t>(i, j))
            {
                ++red_cnt;
            }
        }
    }
    printf("red_cnt: %d\n", red_cnt);
    vel_msg.angular.z = 0;
    if (red_cnt >= red_cnt_threshold)
    {
        vel_msg.linear.x = 0;
        msg.data = true;
        std_msgs::Bool is_align_pub;
        is_align_pub.data = false;
        pub_is_align.publish(is_align_pub);
        geometry_msgs::Twist arm_tight;
        arm_tight.angular.x = 0;
        arm_tight.angular.y = 0;
        arm_tight.angular.z = 0;
        arm_tight.linear.x = 0;
        arm_tight.linear.y = 0.035;
        arm_tight.linear.z = 0;
        pub_grip_arm.publish(arm_tight);
        waitKey(1000);
    }
    else
    {
        if (red_cnt >= slow_threshold)
        {
            vel_msg.linear.x = 1.8;
        }
        else
        {
            vel_msg.linear.x = 3;
        }
        msg.data = false;
    }
    pub.publish(msg);
    pub_cmd_vel.publish(vel_msg);
    cv::imshow("Image", image);
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
    if (is_align)
    {
        ball_approach();
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
}

void alignCallback(const std_msgs::BoolConstPtr &msg)
{
    if (msg->data && (!is_align))
    {
        geometry_msgs::Twist grip_down;
        grip_down.angular.x = 0;
        grip_down.angular.y = 0;
        grip_down.angular.z = 0;
        grip_down.linear.x = 0;
        grip_down.linear.y = 0;
        grip_down.linear.z = -0.055;
        pub_grip_main.publish(grip_down);
        start_time = curr_time;
        geometry_msgs::Twist arm_wide;
        arm_wide.angular.x = 0;
        arm_wide.angular.y = 0;
        arm_wide.angular.z = 0;
        arm_wide.linear.x = 0;
        arm_wide.linear.y = 0;
        arm_wide.linear.z = 0;
        pub_grip_arm.publish(arm_wide);
        waitKey(1000);
    }
    is_align = msg->data;
}

void timeCallback(const std_msgs::Float64ConstPtr &msg)
{
    curr_time = msg->data;
    if (is_align && curr_time > 4 + start_time)
    {
        printf("approach_failed! try again.\n");
        geometry_msgs::Twist vel_msg;
        std_msgs::Bool msg;
        std_msgs::Bool is_align_pub;
        is_align_pub.data = false;
        msg.data = false;
        vel_msg.linear.x = 0;
        vel_msg.angular.z = 0;
        pub_is_align.publish(is_align_pub);
        pub_cmd_vel.publish(vel_msg);
        pub.publish(msg);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "line_distance_node");                                     //init ros node
    ros::NodeHandle nh;                                                              //create node handler
    image_transport::ImageTransport it(nh);                                          //create image transport and connect it to node hanlder
    image_transport::Subscriber sub = it.subscribe("/kinect_rgb", 1, imageCallback); //create subscriber
    image_transport::Subscriber sub_depth = it.subscribe("/kinect_depth", 1, depthCallback);
    ros::Subscriber sub_is_align = nh.subscribe<std_msgs::Bool>("/is_align", 1, alignCallback);
    ros::Subscriber sub_time = nh.subscribe<std_msgs::Float64>("/simulTime", 1, timeCallback);

    pub = nh.advertise<std_msgs::Bool>("/ball_approach_success", 10); //setting publisher
    pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    pub_grip_main = nh.advertise<geometry_msgs::Twist>("/GripperMain/cmd_vel", 10);
    pub_grip_arm = nh.advertise<geometry_msgs::Twist>("/GripperArm/cmd_vel", 10);
    pub_is_align = nh.advertise<std_msgs::Bool>("/is_align", 10);

    std_msgs::Bool is_align_pub;
    is_align_pub.data = false;
    pub_is_align.publish(is_align_pub);

    ros::spin(); //spin.
    return 0;
}
