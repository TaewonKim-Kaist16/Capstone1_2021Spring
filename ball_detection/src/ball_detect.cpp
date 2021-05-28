#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "core_msgs/ball_position.h"
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
float fball_diameter = 0.15;
float fball_radius = 0.075;

const int max_value_H = 360 / 2;
const int max_value = 255;
const int max_kernel_length = 31;
const String window_capture_name = "Video Capture";
const String window_detection_name = "Object Detection";
int color_setting = 1;
const int color_setting_max = 2;
int low_H = 0, low_S = 72, low_V = 52;
int high_H = max_value_H, high_S = max_value, high_V = max_value;
int low_H_red1 = 0, low_H_red2 = 120, low_H_green = 53;
int high_H_red1 = 11, high_H_red2 = max_value_H, high_H_green = 80;

int GB_size_div_two = 2, GB_size = 5;

int erosion_elem = 2;
int erosion_size = 3;
int dilation_elem = 2;
int dilation_size = 3;
const int max_elem = 2;
const int max_kernel_size = 21;
Mat erosion_element, dilation_element;

int con_thresh = 100;
const int max_con_thresh = 255;

RNG rng(12345);

int hough_GB_size_div_two = 3, hough_GB_size = 7;
int hough_GB_10sigma = 15;
int hough_min_dist = 45;
int hough_min_rad = 1, hough_max_rad = 200;
int hough_100dp = 150;
const int max_hough_GB_10sigma = 30;
const int max_hough_min_dist = 200;
const int max_hough_rad = 400;
const int max_hough_100dp = 200;

std::vector<float> pixel2point(Point center, float radius)
{
    std::vector<float> position;
    float x, y, u, v, Xc, Yc, Zc;
    double X, Y, Z, a, b, c;
    double x0, y0, y02, z0, factor;
    double f = 589.3666835307066;
    x = center.x; //.x;// .at(0);
    y = center.y; //.y;//
    u = (x - intrinsic_data[2]) / intrinsic_data[0];
    v = (y - intrinsic_data[5]) / intrinsic_data[4];
    Yc = (intrinsic_data[0] * fball_diameter) / (2 * radius);
    Xc = -u * Yc;
    Zc = -v * Yc;
    X = intrinsic_data[2] - x;
    Z = intrinsic_data[5] - y;
    a = radius * radius;
    b = (a + X * X + Z * Z + f * f) * fball_radius * fball_radius;
    c = (X * X + Z * Z) * fball_radius * fball_radius * fball_radius * fball_radius;
    y02 = (b + std::sqrt(b * b - 4 * a * c)) / (2 * a);
    y0 = 1.062 * std::sqrt(y02) - 0.03457;
    factor = (y0 * y0 - fball_radius * fball_radius) / (y0 * f);
    x0 = X * factor + 0.004;
    z0 = Z * factor;
    Xc = roundf(Xc * 10000) / 10000;
    Yc = roundf(Yc * 10000) / 10000;
    Zc = roundf(Zc * 10000) / 10000;
    position.push_back((float)x0);
    position.push_back((float)y0);
    position.push_back((float)z0);
    return position;
}

static void on_low_H_thresh_trackbar(int, void *)
{
    low_H = min(high_H - 1, low_H);
    setTrackbarPos("Low H", window_detection_name, low_H);
}

static void on_high_H_thresh_trackbar(int, void *)
{
    high_H = max(high_H, low_H + 1);
    setTrackbarPos("High H", window_detection_name, high_H);
}

static void on_low_S_thresh_trackbar(int, void *)
{
    low_S = min(high_S - 1, low_S);
    setTrackbarPos("Low S", window_detection_name, low_S);
}

static void on_high_S_thresh_trackbar(int, void *)
{
    high_S = max(high_S, low_S + 1);
    setTrackbarPos("High S", window_detection_name, high_S);
}

static void on_low_V_thresh_trackbar(int, void *)
{
    low_V = min(high_V - 1, low_V);
    setTrackbarPos("Low V", window_detection_name, low_V);
}

static void on_high_V_thresh_trackbar(int, void *)
{
    high_V = max(high_V, low_V + 1);
    setTrackbarPos("High V", window_detection_name, high_V);
}

static void on_GaussianBlur_Size_trackbar(int, void *)
{
    GB_size = GB_size_div_two * 2 + 1;
    setTrackbarPos("GB Size", window_detection_name, GB_size_div_two);
}

static void on_erosion(int, void *)
{
    int erosion_type = 0;
    if (erosion_elem == 0)
    {
        erosion_type = MORPH_RECT;
    }
    else if (erosion_elem == 1)
    {
        erosion_type = MORPH_CROSS;
    }
    else if (erosion_elem == 2)
    {
        erosion_type = MORPH_ELLIPSE;
    }
    erosion_element = getStructuringElement(erosion_type,
                                            Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                                            Point(erosion_size, erosion_size));
}

static void on_dilation(int, void *)
{
    int dilation_type = 0;
    if (dilation_elem == 0)
    {
        dilation_type = MORPH_RECT;
    }
    else if (dilation_elem == 1)
    {
        dilation_type = MORPH_CROSS;
    }
    else if (dilation_elem == 2)
    {
        dilation_type = MORPH_ELLIPSE;
    }
    dilation_element = getStructuringElement(dilation_type,
                                             Size(2 * dilation_size + 1, 2 * dilation_size + 1),
                                             Point(dilation_size, dilation_size));
}

static void on_contour(int, void *)
{
    return;
}

static void on_hough_GB_size(int, void *)
{
    hough_GB_size = 2 * hough_GB_size_div_two + 1;
    return;
}

static void on_hough_GB_10sigma(int, void *)
{
    return;
}

static void on_hough_min_dist(int, void *)
{
    hough_min_dist = max(1, hough_min_dist);
    return;
}

static void on_hough_min_rad(int, void *)
{
    hough_min_rad = min(hough_min_rad, hough_max_rad - 1);
    setTrackbarPos("Hough min rad:", "Hough Circle", hough_min_rad);
}

static void on_hough_max_rad(int, void *)
{
    hough_max_rad = max(hough_max_rad, hough_min_rad + 1);
    setTrackbarPos("Hough max rad:", "Hough Circle", hough_max_rad);
}

static void on_hough_100dp(int, void *)
{
    return;
}

Mat image_distort;
ros::Publisher pub_red, pub_green;
ros::Publisher pub_markers;

void ball_detect()
{
    Mat image;
    if (!image_distort.data)
    {
        printf("No image data \n");
        return;
    }
    Mat intrinsic = Mat(3, 3, CV_32F, intrinsic_data);
    Mat distCoeffs = Mat(1, 5, CV_32F, distortion_data);
    //undistort(image_distort, image, intrinsic, distCoeffs);
    image = image_distort;
    flip(image, image, 1); // flip the image in horizontal direction
    //imshow("Distort", image_distort);
    //printf("x: %d y: %d\n", image.rows, image.cols);

    //VideoCapture cap(argc > 1 ? atoi(argv[1]) : 0);
    //namedWindow(window_capture_name);
    //namedWindow(window_detection_name);
    //namedWindow("Erosion Demo", WINDOW_AUTOSIZE);
    //namedWindow("Dilation Demo", WINDOW_AUTOSIZE);
    //namedWindow("Contour", WINDOW_AUTOSIZE);
    //namedWindow("Enclosing Circle", WINDOW_AUTOSIZE);
    //namedWindow("Hough Blur", WINDOW_AUTOSIZE);
    //namedWindow("Hough Circle", WINDOW_AUTOSIZE);
    // Trackbars to set thresholds for HSV values s
    //createTrackbar("Color:\n 0: Custom \n 1: Red \n 2: Green", window_detection_name, &color_setting, color_setting_max);
    //createTrackbar("Low H", window_detection_name, &low_H, max_value_H, on_low_H_thresh_trackbar);
    //createTrackbar("High H", window_detection_name, &high_H, max_value_H, on_high_H_thresh_trackbar);
    //createTrackbar("Low S", window_detection_name, &low_S, max_value, on_low_S_thresh_trackbar);
    //createTrackbar("High S", window_detection_name, &high_S, max_value, on_high_S_thresh_trackbar);
    //createTrackbar("Low V", window_detection_name, &low_V, max_value, on_low_V_thresh_trackbar);
    //createTrackbar("High V", window_detection_name, &high_V, max_value, on_high_V_thresh_trackbar);
    //createTrackbar("GB Size", window_detection_name, &GB_size_div_two, max_kernel_length / 2, on_GaussianBlur_Size_trackbar);
    /*createTrackbar("Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", "Erosion Demo",
                   &erosion_elem, max_elem,
                   on_erosion);*/
    /*createTrackbar("Kernel size:\n 2n +1", "Erosion Demo",
                   &erosion_size, max_kernel_size,
                   on_erosion);*/
    /*createTrackbar("Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", "Dilation Demo",
                   &dilation_elem, max_elem,
                   on_dilation);*/
    /*createTrackbar("Kernel size:\n 2n +1", "Dilation Demo",
                   &dilation_size, max_kernel_size,
                   on_dilation);*/
    //createTrackbar("Canny thresh:", "Contour", &con_thresh, max_con_thresh, on_contour);
    //createTrackbar("Hough GB Size:", "Hough Blur", &hough_GB_size_div_two, max_kernel_length / 2, on_hough_GB_size);
    //createTrackbar("Hough GB 10 sigma:", "Hough Blur", &hough_GB_10sigma, max_hough_GB_10sigma, on_hough_GB_10sigma);
    //createTrackbar("Hough 100dp:", "Hough Circle", &hough_100dp, max_hough_100dp, on_hough_100dp);
    //createTrackbar("Hough min dist:", "Hough Circle", &hough_min_dist, max_hough_min_dist, on_hough_min_dist);
    //createTrackbar("Hough min rad:", "Hough Circle", &hough_min_rad, max_hough_rad, on_hough_min_rad);
    //createTrackbar("Hough max rad:", "Hough Circle", &hough_max_rad, max_hough_rad, on_hough_max_rad);

    Mat frame, frame_GB, frame_HSV, frame_threshold, frame_erosion, frame_dilation, frame_contour, frame_circle, frame_hough;
    Mat frame_red1, frame_red2, frame_gray_blur, frame_gray, frame_combined, padded_frame_threshold;
    Mat canny_output;
    std::vector<std::vector<Point>> contours;
    std::vector<Vec4i> hierarchy;
    size_t i, j;
    Scalar contour_color, hough_color;
    std::vector<Vec3f> hough_circles;
    Vec3f hough_circle;
    std::vector<float> ball_pos;
    core_msgs::ball_position msg;         //create a message for ball positions
    visualization_msgs::Marker ball_list; //declare marker
    geometry_msgs::Point p;
    std_msgs::ColorRGBA c;
    int cnt;

    frame = image.clone();
    cnt = 0;
    if (frame.empty())
    {
        return;
    }
    // Apply Gaussian Blur
    GaussianBlur(frame, frame_GB, Size(GB_size, GB_size), 0, 0);
    // Convert from BGR to HSV colorspace
    cvtColor(frame_GB, frame_HSV, COLOR_BGR2HSV);
    // Detect the object based on HSV Range Values
    /*switch (color_setting)
    {
    case 0:
        inRange(frame_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), frame_threshold);
        break;

    case 1:
        inRange(frame_HSV, Scalar(low_H_red1, low_S, low_V), Scalar(high_H_red1, high_S, high_V), frame_red1);
        inRange(frame_HSV, Scalar(low_H_red2, low_S, low_V), Scalar(high_H_red2, high_S, high_V), frame_red2);
        frame_threshold = frame_red1 | frame_red2;
        break;

    case 2:
        inRange(frame_HSV, Scalar(low_H_green, low_S, low_V), Scalar(high_H_green, high_S, high_V), frame_threshold);
        break;

    default:
        break;
    }*/
    frame_combined = image.clone();

    int color_i;
    for (color_i = 0; color_i < 2; ++color_i)
    {
        msg.img_x.clear();
        msg.img_y.clear();
        cnt = 0;
        if (color_i == 0)
        {
            inRange(frame_HSV, Scalar(low_H_red1, low_S, low_V), Scalar(high_H_red1, high_S, high_V), frame_red1);
            inRange(frame_HSV, Scalar(low_H_red2, low_S, low_V), Scalar(high_H_red2, high_S, high_V), frame_red2);
            frame_threshold = frame_red1 | frame_red2;
            c.r = 1.0;
            c.g = 0.0;
            c.b = 0.0;
            c.a = 1.0;
        }
        else if (color_i == 1)
        {
            inRange(frame_HSV, Scalar(low_H_green, low_S, low_V), Scalar(high_H_green, high_S, high_V), frame_threshold);
            c.r = 0.0;
            c.g = 1.0;
            c.b = 0.0;
            c.a = 1.0;
        }
        // Show the frames
        //imshow(window_capture_name, frame);
        //imshow(window_detection_name, frame_threshold);

        copyMakeBorder(frame_threshold, padded_frame_threshold, 10, 10, 10, 10, BORDER_CONSTANT, 0);
        // Erosion, Dialation
        erode(padded_frame_threshold, frame_erosion, erosion_element);
        //imshow("Erosion Demo", frame_erosion);
        dilate(frame_erosion, frame_dilation, dilation_element);
        //imshow("Dilation Demo", frame_dilation);

        // Contour
        Canny(frame_dilation, canny_output, con_thresh, con_thresh * 2);
        //imshow("Canny", canny_output);
        findContours(canny_output, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        std::vector<std::vector<Point>> contours_poly(contours.size());
        std::vector<Point2f> centers(contours.size());
        std::vector<float> radius(contours.size());
        std::vector<Rect> bound_rect(contours.size());
        char buf[256];

        frame_contour = Mat::zeros(canny_output.size(), CV_8UC3);
        frame_circle = image.clone();

        frame_hough = image.clone();

        for (i = 0; i < contours.size(); ++i)
        {
            //contour_color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
            contour_color = Scalar(255, 255, 255);
            approxPolyDP(contours[i], contours_poly[i], 3, true);
            minEnclosingCircle(contours_poly[i], centers[i], radius[i]);
            centers[i].x = centers[i].x - 10;
            centers[i].y = centers[i].y - 10;
            bound_rect[i] = boundingRect(contours_poly[i]);
            drawContours(frame_contour, contours, (int)i, contour_color, 2, LINE_8, hierarchy, 0);
            circle(frame_circle, centers[i], (int)radius[i], Scalar(255, 255, 255), 2);

            // Criterion
            Mat contour_part = Mat::zeros(canny_output.size(), CV_8UC1);
            Mat circle_part = Mat::zeros(image.size(), CV_8UC1);
            drawContours(contour_part, contours, (int)i, contour_color, FILLED, LINE_8, hierarchy, 0);
            circle(circle_part, centers[i], radius[i], Scalar(255, 255, 255), FILLED);
            int contour_area = countNonZero(contour_part);
            int circle_area = countNonZero(circle_part);
            if (circle_area * 0.85 < contour_area)
            {
                ++cnt;
                circle(frame_combined, centers[i], (int)radius[i], Scalar(255, 0, 0), 2);
                ball_pos = pixel2point(centers[i], radius[i]);
                msg.img_x.push_back(ball_pos[0]);
                msg.img_y.push_back(ball_pos[1]);
                p.x = ball_pos[0];
                p.y = ball_pos[1];
                p.z = ball_pos[2];
                ball_list.points.push_back(p);
                ball_list.colors.push_back(c);
                printf("Contour: x: %f, y: %f, z: %f, Color: %d\n", ball_pos[0], ball_pos[1], ball_pos[2], color_i);
                continue;
            }

            // Hough Circle Transform to Part Image
            sprintf(buf, "Part %d", (int)i);
            Mat part(contour_part, bound_rect[i]);
            Mat padded_part;
            hough_circles.clear();
            int x = bound_rect[i].tl().x - bound_rect[i].height - 10;
            int y = bound_rect[i].tl().y - bound_rect[i].width - 10;
            copyMakeBorder(part.clone(), padded_part, bound_rect[i].width, bound_rect[i].width, bound_rect[i].height, bound_rect[i].height, BORDER_CONSTANT, 0);
            if (bound_rect[i].height > 100)
            {
                GaussianBlur(padded_part, frame_gray_blur, Size(31, 31), 2.3, 2.3);
            }
            else if (bound_rect[i].height > 30)
            {
                GaussianBlur(padded_part, frame_gray_blur, Size(31, 31), 2.0, 2.0);
            }
            else
            {
                GaussianBlur(padded_part, frame_gray_blur, Size(31, 31), 1.5, 1.5);
            }
            HoughCircles(frame_gray_blur, hough_circles, HOUGH_GRADIENT, hough_100dp / 100.0, hough_min_dist, 100, 30, 0, 0);
            //printf("%d %d %d\n", x, y, bound_rect[i].height);

            for (j = 0; j < hough_circles.size(); ++j)
            {
                ++cnt;
                hough_circle = hough_circles[j];
                circle(frame_gray_blur, Point(hough_circle[0], hough_circle[1]), hough_circle[2], Scalar(127, 127, 127), 2);
                circle(frame_hough, Point(x + hough_circle[0], y + hough_circle[1]), hough_circle[2], Scalar(255, 255, 255), 2);
                circle(frame_combined, Point(x + hough_circle[0], y + hough_circle[1]), hough_circle[2], Scalar(255, 255, 255), 2);
                ball_pos = pixel2point(Point(x + hough_circle[0], y + hough_circle[1]), hough_circle[2]);
                msg.img_x.push_back(ball_pos[0]);
                msg.img_y.push_back(ball_pos[1]);
                p.x = ball_pos[0];
                p.y = ball_pos[1];
                p.z = ball_pos[2];
                ball_list.points.push_back(p);
                ball_list.colors.push_back(c);
                printf("Hough: x: %f, y: %f, z: %f, color: %d\n", ball_pos[0], ball_pos[1], ball_pos[2], color_i);
            }
            //imshow(buf, frame_gray_blur);
        }
        //printf("%d\n", (int)i);
        //imshow("Contour", frame_contour);
        //imshow("Enclosing Circle", frame_circle);

        // Hough Circle Transform
        /*GaussianBlur(frame_dilation, frame_gray_blur, Size(hough_GB_size, hough_GB_size), 1.5, 1.5);
        imshow("Hough Blur", frame_gray_blur);
        HoughCircles(frame_gray_blur, hough_circles, HOUGH_GRADIENT, hough_100dp / 100.0, hough_min_dist, 100, 30, 0, 0);

        for (i = 0; i < hough_circles.size(); ++i)
        {
            hough_circle = hough_circles[i];
            circle(frame_hough, Point(hough_circle[0], hough_circle[1]), hough_circle[2], Scalar(255, 255, 255), 2);
        }
        //printf("%d\n", (int)i);*/
        //imshow("Hough Circle", frame_hough);

        msg.size = cnt; //adjust the size of message. (*the size of message is varying depending on how many circles are detected)

        // cy = 3.839*(exp(-0.03284*cy))+1.245*(exp(-0.00554*cy));   //convert the position of the ball in camera coordinate to the position in base coordinate. It is related to the calibration process. You shoould modify this.
        // cx = (0.002667*cy+0.0003)*cx-(0.9275*cy+0.114);

        cv::waitKey(1);
        if (color_i == 0)
        {
            pub_red.publish(msg); //publish a message
        }
        else if (color_i == 1)
        {
            pub_green.publish(msg);
        }
    }
    imshow("Combined", frame_combined);

    ball_list.header.frame_id = "map";         //set the frame
    ball_list.header.stamp = ros::Time::now(); //set the header. without it, the publisher may not publish.
    ball_list.ns = "balls";                    //name of markers
    ball_list.action = visualization_msgs::Marker::ADD;
    ball_list.pose.position.x = 0; //the transformation between the frame and camera data, just set it (0,0,0,0,0,0) for (x,y,z,roll,pitch,yaw)
    ball_list.pose.position.y = 0;
    ball_list.pose.position.z = 0;
    ball_list.pose.orientation.x = 0;
    ball_list.pose.orientation.y = 0;
    ball_list.pose.orientation.z = 0;
    ball_list.pose.orientation.w = 1.0;

    ball_list.id = 0;                                         //set the marker id. if you use another markers, then make them use their own unique ids
    ball_list.type = visualization_msgs::Marker::SPHERE_LIST; //set the type of marker

    ball_list.scale.x = fball_diameter; //set the radius of marker   1.0 means 1.0m, 0.001 means 1mm
    ball_list.scale.y = fball_diameter;
    ball_list.scale.z = fball_diameter;
    pub_markers.publish(ball_list); //publish a marker message
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        image_distort = cv_bridge::toCvShare(msg, "bgr8")->image; //transfer the image data into buffer
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
    ball_detect(); //proceed ball detection
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ball_detect_node");                                           //init ros nodd
    ros::NodeHandle nh;                                                                  //create node handler
    image_transport::ImageTransport it(nh);                                              //create image transport and connect it to node hnalder
    image_transport::Subscriber sub = it.subscribe("/kinect_rgb_top", 1, imageCallback); //create subscriber

    pub_red = nh.advertise<core_msgs::ball_position>("/red_position", 100); //setting publisher
    pub_green = nh.advertise<core_msgs::ball_position>("/green_position", 100);
    pub_markers = nh.advertise<visualization_msgs::Marker>("/balls", 1);

    ros::spin(); //spin.
    return 0;
}
