#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "core_msgs/plane_info.h"
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
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

RNG rng(12345);

std::vector<float> pixel2point(Point center, float distance)
{
    std::vector<float> position;
    float x, y, u, v, Xc, Yc, Zc;
    double X, Y, Z;
    double x0, y0, z0, factor;
    double f = 589.3666835307066;
    x = center.x; //.x;// .at(0);
    y = center.y; //.y;//
    X = intrinsic_data[2] - x;
    Z = intrinsic_data[5] - y;
    y0 = distance;
    factor = y0 / f;
    x0 = X * factor;
    z0 = Z * factor;
    position.push_back((float)x0);
    position.push_back((float)y0);
    position.push_back((float)z0);
    return position;
}

double slopes2angle(std::vector<float> slopes)
{
    int sign = 1;
    if (slopes[1] < 0)
    {
        sign = -1;
    }
    double slope = sign * std::asin(slopes[2]) * 180 / 3.1415926535;
    return slope;
}

Mat image_distort, image_brg, mask;
ros::Publisher pub;

void image_filter()
{
    Mat gray;
    cvtColor(image_brg, gray, CV_BGR2GRAY);
    threshold(gray, mask, 50, 255, THRESH_BINARY);
    //cv::imshow("Binary", mask);
    cv::waitKey(1);
}

void line_distance()
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
    int rows, cols;
    rows = image.rows;
    cols = image.cols;
    int i, j;
    double sinvalue, dx, dy, dz, slope;
    Mat image_line = image_brg.clone();
    std::vector<float> prev_point, curr_point;
    int flag = 2;
    int sign;
    double mean_slope_x, mean_slope_y, mean_slope_z;
    double mean_slope = 180;
    int slope_cnt;
    std::vector<float> slope_start;
    std::vector<std::vector<float>> left_start, right_start;
    std::vector<std::vector<float>> left_slopes, right_slopes;
    std::vector<float> slopes;
    for (i = rows - 1; i >= 0; i = i - 15)
    {
        for (j = 1; j < cols; ++j)
        {
            if (mask.at<uint8_t>(i, j - 1) != 0 && mask.at<uint8_t>(i, j) == 0)
            {
                flag = 1;
                image_line.at<Vec3b>(i, j) = Vec3b(0, 0, 255);
                curr_point = pixel2point(Point(j, i), image.at<float>(i, j));
                if (!prev_point.empty())
                {
                    dx = curr_point[0] - prev_point[0];
                    dy = curr_point[1] - prev_point[1];
                    dz = curr_point[2] - prev_point[2];
                    if (dy < 0)
                    {
                        sign = -1;
                    }
                    else
                    {
                        sign = 1;
                    }
                    slopes.clear();
                    sinvalue = dx / std::sqrt(dx * dx + dy * dy + dz * dz);
                    slopes.push_back(sinvalue);
                    sinvalue = dy / std::sqrt(dx * dx + dy * dy + dz * dz);
                    slopes.push_back(sinvalue);
                    sinvalue = dz / std::sqrt(dx * dx + dy * dy + dz * dz);
                    slopes.push_back(sinvalue);
                    slope = sign * std::asin(sinvalue) * 180 / 3.1415926535;
                    //printf("i: %d, dx: %f, dy: %f, dz: %f, slope(deg): %f\n", i, dx, dy, dz, slope);
                    if (mean_slope == 180)
                    {
                        slope_start = curr_point;
                        mean_slope_x = slopes[0];
                        mean_slope_y = slopes[1];
                        mean_slope_z = slopes[2];
                        mean_slope = slope;
                        slope_cnt = 1;
                    }
                    else if (mean_slope - 15 > slope || mean_slope + 15 < slope)
                    {
                        printf("Mean Slope Left(deg): %f, i: %d\n", mean_slope, i);
                        left_start.push_back(slope_start);
                        slopes.clear();
                        slopes.push_back(mean_slope_x);
                        slopes.push_back(mean_slope_y);
                        slopes.push_back(mean_slope_z);
                        left_slopes.push_back(slopes);
                        mean_slope = 180;
                    }
                    else
                    {
                        mean_slope_x = mean_slope_x * slope_cnt + slopes[0];
                        mean_slope_y = mean_slope_y * slope_cnt + slopes[1];
                        mean_slope_z = mean_slope_z * slope_cnt + slopes[2];
                        mean_slope = mean_slope * slope_cnt + slope;
                        ++slope_cnt;
                        mean_slope_x = mean_slope_x / slope_cnt;
                        mean_slope_y = mean_slope_y / slope_cnt;
                        mean_slope_z = mean_slope_z / slope_cnt;
                        mean_slope = mean_slope / slope_cnt;
                    }
                }
                prev_point = curr_point;
            }
        }
        if (!flag)
        {
            break;
        }
        if (flag != 2)
        {
            flag = 0;
        }
    }
    if (mean_slope != 180)
    {
        printf("Mean Slope Left(deg): %f, i: %d\n", mean_slope, i);
        left_start.push_back(slope_start);
        slopes.clear();
        slopes.push_back(mean_slope_x);
        slopes.push_back(mean_slope_y);
        slopes.push_back(mean_slope_z);
        left_slopes.push_back(slopes);
    }
    prev_point.clear();
    flag = 2;
    mean_slope = 180;
    for (i = rows - 1; i >= 0; i = i - 15)
    {
        for (j = 0; j < cols - 1; ++j)
        {
            if (mask.at<uint8_t>(i, j) == 0 && mask.at<uint8_t>(i, j + 1) != 0)
            {
                flag = 1;
                image_line.at<Vec3b>(i, j) = Vec3b(0, 255, 0);
                curr_point = pixel2point(Point(j, i), image.at<float>(i, j));
                if (!prev_point.empty())
                {
                    dx = curr_point[0] - prev_point[0];
                    dy = curr_point[1] - prev_point[1];
                    dz = curr_point[2] - prev_point[2];
                    if (dy < 0)
                    {
                        sign = -1;
                    }
                    else
                    {
                        sign = 1;
                    }
                    slopes.clear();
                    sinvalue = dx / std::sqrt(dx * dx + dy * dy + dz * dz);
                    slopes.push_back(sinvalue);
                    sinvalue = dy / std::sqrt(dx * dx + dy * dy + dz * dz);
                    slopes.push_back(sinvalue);
                    sinvalue = dz / std::sqrt(dx * dx + dy * dy + dz * dz);
                    slopes.push_back(sinvalue);
                    slope = sign * std::asin(sinvalue) * 180 / 3.1415926535;
                    //printf("i: %d, dx: %f, dy: %f, dz: %f, slope(deg): %f\n", i, dx, dy, dz, slope);
                    if (mean_slope == 180)
                    {
                        slope_start = curr_point;
                        mean_slope_x = slopes[0];
                        mean_slope_y = slopes[1];
                        mean_slope_z = slopes[2];
                        mean_slope = slope;
                        slope_cnt = 1;
                    }
                    else if (mean_slope - 15 > slope || mean_slope + 15 < slope)
                    {
                        printf("Mean Slope Right(deg): %f, i: %d\n", mean_slope, i);
                        right_start.push_back(slope_start);
                        slopes.clear();
                        slopes.push_back(mean_slope_x);
                        slopes.push_back(mean_slope_y);
                        slopes.push_back(mean_slope_z);
                        right_slopes.push_back(slopes);
                        mean_slope = 180;
                    }
                    else
                    {
                        mean_slope_x = mean_slope_x * slope_cnt + slopes[0];
                        mean_slope_y = mean_slope_y * slope_cnt + slopes[1];
                        mean_slope_z = mean_slope_z * slope_cnt + slopes[2];
                        mean_slope = mean_slope * slope_cnt + slope;
                        ++slope_cnt;
                        mean_slope_x = mean_slope_x / slope_cnt;
                        mean_slope_y = mean_slope_y / slope_cnt;
                        mean_slope_z = mean_slope_z / slope_cnt;
                        mean_slope = mean_slope / slope_cnt;
                    }
                }
                prev_point = curr_point;
            }
        }
        if (!flag)
        {
            break;
        }
        if (flag != 2)
        {
            flag = 0;
        }
    }
    if (mean_slope != 180)
    {
        printf("Mean Slope Right(deg): %f, i: %d\n", mean_slope, i);
        right_start.push_back(slope_start);
        slopes.clear();
        slopes.push_back(mean_slope_x);
        slopes.push_back(mean_slope_y);
        slopes.push_back(mean_slope_z);
        right_slopes.push_back(slopes);
    }
    // Plane and Distance Calculation
    double left_slope, right_slope;
    double a, b, c, x0, y0, z0;
    double distance;
    std::vector<std::vector<float>> temp;
    std::vector<float> vectora, vectorb, normal_vector;
    std::vector<float> distances, final_slopes;
    core_msgs::plane_info msg;
    j = 0;
    int k;
    if (left_slopes.size() > right_slopes.size())
    {
        temp = left_slopes;
        left_slopes = right_slopes;
        right_slopes = temp;
        temp = left_start;
        left_start = right_start;
        right_start = temp;
    }
    for (i = 0; i < left_slopes.size(); ++i)
    {
        left_slope = slopes2angle(left_slopes[i]);
        for (; j < right_slopes.size(); ++j)
        {
            right_slope = slopes2angle(right_slopes[j]);
            if (left_slope + 7 > right_slope && left_slope - 7 < right_slope)
            {
                vectora.clear();
                vectorb.clear();
                normal_vector.clear();
                for (k = 0; k < 3; ++k)
                {
                    vectora.push_back((left_slopes[i][k] + right_slopes[j][k]) / 2);
                    vectorb.push_back((left_start[i][k] - right_start[j][k]));
                }
                for (k = 0; k < 3; ++k)
                {
                    normal_vector.push_back(vectora[(1 + k) % 3] * vectorb[(2 + k) % 3] - vectora[(2 + k) % 3] * vectorb[(1 + k) % 3]);
                }
                a = normal_vector[0];
                b = normal_vector[1];
                c = normal_vector[2];
                x0 = left_start[i][0];
                y0 = left_start[i][1];
                z0 = left_start[i][2];
                distance = (a * x0 + b * y0 + c * z0) / std::sqrt(a * a + b * b + c * c);
                if (distance < 0)
                {
                    distance = -distance;
                }
                printf("Start: x: %f, y: %f, z: %f, d: %f\n", x0, y0, z0, std::sqrt(x0 * x0 + y0 * y0 + z0 * z0));
                printf("Distance: %f\n", distance);
                msg.distance.push_back(distance);
                msg.slope.push_back((left_slope + right_slope) / 2);
                distances.push_back(distance);
                final_slopes.push_back((left_slope + right_slope) / 2);
                ++j;
                break;
            }
        }
    }
    msg.size = distances.size();
    pub.publish(msg);
    //cv::imshow("Depth", image);
    //cv::imshow("HaHaHa", image_brg);
    cv::imshow("HeHeHe", image_line);
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
    image_filter();
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
    line_distance();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "line_distance_node");                                     //init ros nodd
    ros::NodeHandle nh;                                                              //create node handler
    image_transport::ImageTransport it(nh);                                          //create image transport and connect it to node hnalder
    image_transport::Subscriber sub = it.subscribe("/kinect_rgb", 1, imageCallback); //create subscriber
    image_transport::Subscriber sub_depth = it.subscribe("/kinect_depth", 1, depthCallback);

    pub = nh.advertise<core_msgs::plane_info>("/plane_info", 100); //setting publisher

    ros::spin(); //spin.
    return 0;
}
