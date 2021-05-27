#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "core_msgs/ball_position.h"
#include "core_msgs/dist_center.h"
#include "opencv2/opencv.hpp"
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

using namespace cv;
using namespace std;

Mat buffer;
ros::Publisher pub;


void line_trace();
void imageCallback(const sensor_msgs::ImageConstPtr& msg);


int main(int argc, char **argv)
{
   ros::init(argc, argv, "line_tracing_node"); //init ros node
   ros::NodeHandle nh; //create node handler
   image_transport::ImageTransport it(nh); //create image transport and connect it to node hnalder
   image_transport::Subscriber sub = it.subscribe("/kinect_rgb", 1, imageCallback); //create subscriber

   pub = nh.advertise<core_msgs::dist_center>("/distance", 100); //setting publisher

   ros::spin(); //spin.
   return 0;
}

void line_trace(){
    int rows;
    int cols;
//    int leftang=0;
//    int rightang=0;
    Mat gray;  //assign a memory to convert into grayscale images
    Mat blur_img; // assign a memory for gaussian blur (preprocessing)
    vector<int>centers(16); // 16 center points of segmented guidelines
    double thresh;
    cvtColor(buffer, gray, CV_RGB2GRAY);
    flip(gray, gray, 1);
    GaussianBlur(gray, blur_img, Size(5, 5), 0);
    thresh = threshold(blur_img, buffer,50, 255, THRESH_BINARY);
    rows = buffer.rows;
    cols = buffer.cols;

    cvtColor(buffer, buffer, CV_GRAY2BGR);
    for (int i =0; i <rows; i++){
        buffer.at<Vec3b>(i, cols/2) = Vec3b(0,0,255);
        buffer.at<Vec3b>(i, cols/2+1) = Vec3b(0,0,255);
    }   //center_datum: red line

    for (int i =0; i <rows; i=i+rows/16){
        int leftend = 0;
        int rightend = 0;
        int center;
        int point_on;
        for (int j = 1; j < cols-1; j ++){
            if (buffer.at<Vec3b>(i, j-1) == Vec3b(0,0,0) && buffer.at<Vec3b>(i, j) != Vec3b(0,0,0))
                leftend = j;
            if (buffer.at<Vec3b>(i, j) != Vec3b(0,0,0) && buffer.at<Vec3b>(i, j+1) == Vec3b(0,0,0))
                rightend = j;
        }
        if ((leftend+rightend) != 0)
            center = (leftend+rightend)/2;
        else
            center = rows/2;
            if ((leftend == 0) and (rightend == 0))
                point_on = 0;
            else
                point_on = 1;
        if (point_on == 1){
            buffer.at<Vec3b>(i, center) = Vec3b(0,255, 0);
            buffer.at<Vec3b>(i+1, center) = Vec3b(0,255, 0);
        }
        centers[i/(rows/16)]= center - rows/2;
    }
//    for (int k = 0; k<16; k++){
//        if (centers[k]<0)
//            leftang++;
//        else if (centers[k]>0)
//            rightang++;
//    }
    core_msgs::dist_center msg;
    msg.size =centers.size(); //adjust the size of message. (*the size of message is varying depending on how many circles are detected)
    msg.dist.resize(centers.size());  //adjust the size of array

    for (int k = 0; k<16; k++){
        msg.dist[k]=centers[k];
    }
    cv::namedWindow("bottom_view", WINDOW_AUTOSIZE);
    cv::imshow("bottom_view", buffer);  //show the image with a window
    cv::waitKey(1);
    pub.publish(msg);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

   try
   {
     buffer = cv_bridge::toCvShare(msg, "bgr8")->image;  //transfer the image data into buffer
   }
   catch (cv_bridge::Exception& e)
   {
     ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
   }
   line_trace(); //proceed line_tracing
}

