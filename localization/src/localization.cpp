#include <ros/ros.h>
#include "string.h"
#include <sstream>
#include <cmath>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <algorithm>
#include "localization/robot_position.h"
#include "localization/multi_position.h"
#include "core_msgs/ball_position.h"

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>

using namespace std;

// Global variable
sensor_msgs::PointCloud2 msg_cloud;
sensor_msgs::PointCloud2 msg_obstacle;
geometry_msgs::Twist vel_msg;
localization::multi_position obstacle_data;
localization::multi_position red_balls_data;
localization::multi_position green_ball_data;
float timestamp_start, timestamp_current;
float ang_error, ang_error_pre, ang_error_dot, ang_error_tot;

int mode = 1;
// int wait_count = 0;

struct WALL {
    float x,y,x_dir,y_dir;
};

struct RELATIVE_POSITION {
    WALL walls[4];
    geometry_msgs::Point obstacles[4]; // Sometimes the goal post is detected
    geometry_msgs::Point red_balls[6];
    geometry_msgs::Point green_ball;
    geometry_msgs::Point reference;
    geometry_msgs::Point crossing_points[4];
};

struct ABSOLUTE_POSITION {
    geometry_msgs::Point pose;
    float angle;
};

ABSOLUTE_POSITION robot_geometry;
ABSOLUTE_POSITION obstacle_geometry[3];
RELATIVE_POSITION relative_position;

void init_vel() {
    vel_msg.linear.x = 0;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = 0;
};


void get_wall_coeff(WALL wall,float coeffs[]) {
    float a = wall.x_dir;
    float b = wall.y_dir;
    float c = b*wall.x-a*wall.y;
    coeffs[0] = a, coeffs[1] = b, coeffs[2] =c;
}

geometry_msgs::Point get_crossing_point(float coeffs1[],float coeffs2[]) {
    float a1, b1, c1, a2, b2, c2;
    a1 = coeffs1[0], b1 = coeffs1[1], c1 = coeffs1[2];
    a2 = coeffs2[0], b2 = coeffs2[1], c2 = coeffs2[2];

    float det = a1*b2-b1*a2;

    geometry_msgs::Point p;
    p.x = (1/det)*(a1*c2-a2*c1);
    p.y = (1/det)*(b1*c2-b2*c1);

    return p;
}

void transform_rel2abs(int idx1, int idx2, geometry_msgs::Point ref_point1, geometry_msgs::Point ref_point2) {
    geometry_msgs::Point abs_p[4];

    abs_p[0].x = 0; abs_p[0].y = 0;
    abs_p[1].x = 3; abs_p[1].y = 0;
    abs_p[2].x = 0; abs_p[2].y = 5;
    abs_p[3].x = 3; abs_p[3].y = 5;

    geometry_msgs::Point abs_p1 = abs_p[idx1];
    geometry_msgs::Point abs_p2 = abs_p[idx2];

    /*
    vec_abs = M * vec_rel

    M = [r1 -r2  p1;
         r2  r1  p2;
         0   0   1]

    A = x1_abs-x2_abs, B = y1_abs-y2_abs, C = x1_rel-x2_rel, D = y1_rel-y2_rel
    r1 = (AC+BD)/(C^2+D^2), r2 = (BC-AD)/(C^2+D^2)
    p1 = x1_abs - r1*x1_rel + r2*y1_rel
    p2 = y1_abs -r2*x1_rel - r1*y1_rel

    inv(M) = [r1  r2  -p1*r1-p2*r2;
             -r2  r1   p1*r2-p2*r1
              0   0       1        ]

    */

    float A = abs_p1.x-abs_p2.x;
    float B = abs_p1.y-abs_p2.y;
    float C = ref_point1.x - ref_point2.x;
    float D = ref_point1.y-ref_point2.y;

    float r1 = (A*C+B*D)/(pow(C,2)+pow(D,2));
    float r2 = (B*C-A*D)/(pow(C,2)+pow(D,2));
    float p1 = abs_p1.x - r1*ref_point1.x + r2*ref_point1.y;
    float p2 = abs_p1.y - r2*ref_point1.x -r1*ref_point1.y;

    float previous_x = robot_geometry.pose.x;
    float previous_y = robot_geometry.pose.y;
    float dist = sqrt(pow(p1-previous_x,2)+pow(p2-previous_y,2));

    if (dist < 0.4) {
        robot_geometry.pose.x = p1;
        robot_geometry.pose.y = p2;
        robot_geometry.angle = atan2(r2,r1);

        for (int i=0; i<4; i++) {
            float x_abs = abs_p[i].x;
            float y_abs = abs_p[i].y;
            relative_position.crossing_points[i].x = r1*x_abs + r2*y_abs - p1*r1 - p2*r2;
            relative_position.crossing_points[i].y = -r2*x_abs + r1*y_abs + p1*r2 - p2*r1;
        }

        cout << "robot geometry ----------" << endl 
            << "pose x,y: " << robot_geometry.pose.x << " " << robot_geometry.pose.y << endl
            << "angle : " << robot_geometry.angle << endl << endl;

        obstacle_data.num = 0;
        obstacle_data.data.clear();
        cout << "obstacle ----------------" << endl;
        for (int j=0;j<4;j++) {
            if (relative_position.obstacles[j].x == 0 && relative_position.obstacles[j].y == 0) { continue; }

            // cout << j << " th : " << relative_position.obstacles[j].x << " " << relative_position.obstacles[j].y << endl;
            float obs_abs_x = r1*relative_position.obstacles[j].x - r2*relative_position.obstacles[j].y +p1;
            float obs_abs_y = r2*relative_position.obstacles[j].x + r1*relative_position.obstacles[j].y +p2;

            if (obs_abs_x < 0 || obs_abs_x > 3) { continue; }
            else if (obs_abs_y < 0 || obs_abs_y > 5) { continue; }
            else if ( (obs_abs_x > 1.4 || obs_abs_x < 1.4) && (obs_abs_y > 4.6) ) { continue; }

            cout << j << " th : " << obs_abs_x << " " << obs_abs_y << endl;
     
            geometry_msgs::Point obs_p;
            obs_p.x = obs_abs_x;
            obs_p.y = obs_abs_y;            

            obstacle_data.data.push_back(obs_p);
            obstacle_data.num ++;
        }
        cout << endl;
        
        red_balls_data.num = 0;
        red_balls_data.data.clear();
        cout << "red balls ---------------" << endl;
        for (int k=0;k<6;k++) {
            if (relative_position.red_balls[k].x < -10 || relative_position.red_balls[k].y < -10) { continue; }
            if (relative_position.red_balls[k].x == 0 && relative_position.red_balls[k].y == 0) { continue; }

            // TODO : Compare with real position
            // cout << k << " th : " << relative_position.red_balls[k].x << " " << relative_position.red_balls[k].y << endl;
            float rb_abs_x = r1*relative_position.red_balls[k].x - r2*relative_position.red_balls[k].y +p1;
            float rb_abs_y = r2*relative_position.red_balls[k].x + r1*relative_position.red_balls[k].y +p2;

            if (rb_abs_x < 0 || rb_abs_x > 3) { continue; }
            else if (rb_abs_y < 0 || rb_abs_y > 5) { continue; }

            cout << k << " th : " << rb_abs_x << " " << rb_abs_y << endl;

            geometry_msgs::Point rb_p;
            rb_p.x = rb_abs_x;
            rb_p.y = rb_abs_y;            

            red_balls_data.data.push_back(rb_p);
            red_balls_data.num ++;
        }
        cout << endl;

        green_ball_data.num = 0;
        green_ball_data.data.clear();
        cout << "green balls -------------" << endl;
        if (relative_position.green_ball.x < -10 || relative_position.green_ball.y < -10) { }
        else if (relative_position.green_ball.x == 0 && relative_position.green_ball.y == 0) { }
        else {
            // TODO : Compare with real position
            // cout << relative_position.green_ball.x << " " << relative_position.green_ball.y << endl;
            float gb_abs_x = r1*relative_position.green_ball.x - r2*relative_position.green_ball.y +p1;
            float gb_abs_y = r2*relative_position.green_ball.x + r1*relative_position.green_ball.y +p2;

            if (gb_abs_x < 0 || gb_abs_x > 3) {}
            else if (gb_abs_y < 0 || gb_abs_y > 5) {}
            else {
                cout << gb_abs_x << " " << gb_abs_y << endl;

                geometry_msgs::Point gb_p;
                gb_p.x = gb_abs_x;
                gb_p.y = gb_abs_y;            

                green_ball_data.data.push_back(gb_p);
                green_ball_data.num ++;
            }
        }
        cout << endl;
    }
}

void mode_1_rearrage(double th) {

    cout << "mode1 rearranging..." << endl;

    float kp=0.1,kd=0.0001,ki=0.01;

    cout << "current time : " << timestamp_current<< " start time : " << timestamp_start<< endl;
    float dt = max(timestamp_current- timestamp_start,(float)0.001);

    ang_error = th - 90;
    ang_error_dot = (ang_error - ang_error_pre) / dt;
    ang_error_tot = ang_error * dt + ang_error_tot;

    if (abs(th-90) > 3) {

        float ang_vel = kp*ang_error+kd*ang_error_dot+ki*ang_error_tot;
        if (ang_vel >= 0) { vel_msg.angular.z = min(ang_vel, (float)4); }
        else { vel_msg.angular.z = max(ang_vel, (float)-1); }

        timestamp_start = timestamp_current;
    
        cout << "dt : " << dt << endl;
        cout << "error : " << ang_error << " | " << kp*ang_error << endl;
        cout << "error dot : " << ang_error_dot << " | " << kd*ang_error_dot << endl;
        cout << "error tot : " << ang_error_tot << " | " << ki*ang_error_tot << endl;
        cout << "ang vel : " << vel_msg.angular.z << endl;

    }

    else {
        if (timestamp_current- timestamp_start> 0.25) {
            mode = 2;
            timestamp_start = timestamp_current;
        }
    }
};

void mode_2_goforward(double th) {
    cout << "mode2 go forward..." << endl;
    
    float dt = 4;
    if (timestamp_current- timestamp_start< dt) {
        vel_msg.linear.x = 4;
        vel_msg.angular.z = (th-90)*0.5;
    }
    else {
        vel_msg.linear.x = 0;
        vel_msg.angular.z = 0;
        mode = 3;
    }
    
}

void mode_3_setRef() {
    cout << "mode3 set reference point..."<<endl;

    WALL parallel_wall;
    WALL verticle_wall;

    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;

    int y_p = -100;
    for (int i=1;i<4;i++) {
        float a = relative_position.walls[i].x_dir;
        float b = relative_position.walls[i].y_dir;
        float th2 = atan2(b,a)*180/M_PI;

        if ((abs(th2)-10) < 0 || (abs(th2-180)-10) < 0) {
            cout << i << " th verticle wall" << endl;
            cout << "y value : " << relative_position.walls[i].y << endl;
            if (y_p < relative_position.walls[i].y && relative_position.walls[i].y < 0) {
                verticle_wall = relative_position.walls[i];
                y_p = (int) relative_position.walls[i].y;
            }
        }
        else {
            parallel_wall = relative_position.walls[i];
        }
    }

    cout << "verticle wall y : " << verticle_wall.y << endl;

    float coeff_ver[3];
    get_wall_coeff(verticle_wall,coeff_ver);

    float coeff_par[3];
    get_wall_coeff(parallel_wall,coeff_par);

    relative_position.reference = get_crossing_point(coeff_ver,coeff_par);


    robot_geometry.pose.x = -relative_position.reference.x;
    robot_geometry.pose.y = -relative_position.reference.y;
    robot_geometry.angle = 0;

    /*
    point1 = (0,0), point2 = (3,0), point3 = (0,5), point4 = (3,5)
    */

    relative_position.crossing_points[0] = relative_position.reference;

    relative_position.crossing_points[1].x = relative_position.reference.x+3;
    relative_position.crossing_points[1].y = relative_position.reference.y;
    
    relative_position.crossing_points[2].x = relative_position.reference.x;
    relative_position.crossing_points[2].y = relative_position.reference.y+5;
    
    relative_position.crossing_points[3].x = relative_position.reference.x+3;
    relative_position.crossing_points[3].y = relative_position.reference.y+5;

    // cout << "crossing_points" << endl << "0 : " << relative_position.crossing_points[0] << endl
    //                                   << "1 : " << relative_position.crossing_points[1] << endl
    //                                   << "2 : " << relative_position.crossing_points[2] << endl
    //                                   << "3 : " << relative_position.crossing_points[3] << endl;
}


void mode_4_localizaiont() {
    cout << "mode4 localizate robot..."<<endl;

    float coeffs[4][3];
    for (int i=0;i<4;i++) {
        get_wall_coeff(relative_position.walls[i],coeffs[i]);
    }

    float dist;
    geometry_msgs::Point ref_point1;
    geometry_msgs::Point ref_point2;

    float min_dist1 = 100;
    float min_dist2 = 120;
    int idx1 = 0; int idx2 = 0;
    for (int i = 0; i<4; i++) {
        // cout << i << "crossing_point" << endl << relative_position.crossing_points[i] <<endl;
        geometry_msgs::Point old_ref = relative_position.crossing_points[i];
        for (int j=0;j<4;j++) {
            for (int k=j+1;k<4;k++) {
                geometry_msgs::Point p = get_crossing_point(coeffs[j],coeffs[k]);
                float dist_from_robot = sqrt(pow(p.x,2)+pow(p.y,2));
                if (dist_from_robot > 5) { continue; }

                float dist_from_old = sqrt(pow(p.x-old_ref.x,2)+pow(p.y-old_ref.y,2));

                if (dist_from_old < min_dist1) {
                    ref_point2 = ref_point1;
                    min_dist2 = min_dist1;
                    idx2 = idx1;
                    ref_point1 = p;
                    min_dist1 = dist_from_old;
                    idx1 = i;
                }
                else if (dist_from_old < min_dist2) {
                    ref_point2 = p;
                    min_dist2 = dist_from_old;
                    idx2 = i;
                }

            }
        }
    }
    // cout << "idx1 : " << idx1 << endl << "old : " << endl << relative_position.crossing_points[idx1] << endl
    //                                   << "new : " << endl << ref_point1 << endl
    //                                   << "dist :" << endl << min_dist1 << endl;
    // cout << "idx2 : " << idx2 << endl << "old : " << endl << relative_position.crossing_points[idx2] << endl
    //                                   << "new : " << endl << ref_point2 << endl
    //                                   << "dist :" << endl << min_dist2 << endl;                                  

    transform_rel2abs(idx1,idx2,ref_point1,ref_point2);

}

void GenerateMap(){

    /*
    (x-x1)/a1 = (y-y1)/b1
    (x-x2)/a2 = (y-y2)/b2
    */

    float a1 = relative_position.walls[0].x_dir;
    float b1 = relative_position.walls[0].y_dir;
    float c1 = b1*relative_position.walls[0].x-a1*relative_position.walls[0].y;

    // Mode1 rearrange the robot with right wall
    double th = atan2(b1,a1)*180/M_PI;
    if (th < 0) {
        th = th + 180;
    }

    if (mode == 1) {
        mode_1_rearrage(th);
    }
    else if (mode == 2) {
        mode_2_goforward(th);
    }
    else if (mode == 3) {
        mode_3_setRef();
        mode = 4;
    }
    else if (mode == 4) {
        mode_4_localizaiont();
    }
}



void lidar_cb(sensor_msgs::LaserScan msg){

    // angle in radian
    float angle_min = msg.angle_min;
    float angle_max = msg.angle_max;
    float angle_increment = msg.angle_increment;
    std::vector<float> range = msg.ranges;

    // size of range vector
    int len = range.size();
    float angle_temp;

    /// 1. LaserScan msg to PCL::PointXYZ

    // initializae pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr total_cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr center_points (new pcl::PointCloud<pcl::PointXYZ>);

    // Create the extract object for removal of infinite ditance points
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointIndices::Ptr inf_points(new pcl::PointIndices());

    cloud->is_dense = false;
    cloud->width = len;
    cloud->height = 1;
    cloud->points.resize(len);

    // fill the pointcloud
    for(int i = 0; i < len; i++){
        angle_temp = angle_min + i*angle_increment;
        if (std::isinf(range[i])==false){

            cloud->points[i].x = range[i]*cos(angle_temp);
            cloud->points[i].y = range[i]*sin(angle_temp);
            cloud->points[i].z = 0;
        }
        else{
            // indices of infinite distance points
            inf_points->indices.push_back(i);
        }
    }

    // Remove infinite distance points from cloud
    extract.setInputCloud(cloud);
    extract.setIndices(inf_points);
    extract.setNegative(true);
    extract.filter(*cloud);
    // std::cout << "PointCloud before filtering has: " << cloud->size () << " data points." << std::endl;

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*cloud_filtered);
    // std::cout << "PointCloud after filtering has: " << cloud_filtered->size ()  << " data points." << std::endl;

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr total_cloud_line (new pcl::PointCloud<pcl::PointXYZ> ());

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_LINE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.03);

    for(int i = 0; i < 4; i++){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line (new pcl::PointCloud<pcl::PointXYZ> ());
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_line);
        *total_cloud_line += *cloud_line;
        // std::cout << "PointCloud representing the planar component: " << cloud_line->size () << " data points." << std::endl;

        // std::cout << i+1 << " line coefficients ------------" << endl
        //                  << "  points on x : " << coefficients->values[0] <<endl
        //                  << "  points on y : " << coefficients->values[1] <<endl
        //                  << "  direction x : " << coefficients->values[3] <<endl
        //                  << "  direction y : " << coefficients->values[4] << endl;

        relative_position.walls[i].x = coefficients->values[0];
        relative_position.walls[i].y = coefficients->values[1];
        relative_position.walls[i].x_dir = coefficients->values[3];
        relative_position.walls[i].y_dir = coefficients->values[4];

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);
        *cloud_filtered = *cloud_f;     
    }

    if (mode == 4) {
        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (cloud_filtered);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (0.03);
        ec.setMinClusterSize (5);
        ec.setMaxClusterSize (40);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud_filtered);
        ec.extract (cluster_indices);

        int j = 0;
    
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            pcl::CentroidPoint<pcl::PointXYZ> centroid;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
            for (const auto& idx : it->indices)
            {
                centroid.add((*cloud_filtered)[idx]);
                cloud_cluster->push_back ((*cloud_filtered)[idx]); //*
                total_cloud_cluster->push_back ((*cloud_filtered)[idx]); //*
            }

            pcl::PointXYZ center_p;
            centroid.get(center_p);
            center_points->push_back(center_p);

            // std::cout << j << " PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
            // std::cout << j << " PointCloud x, y : " << center_p.x << ", " << center_p.y << std::endl;
            
            relative_position.obstacles[j].x = center_p.x;
            relative_position.obstacles[j].y = center_p.y;


            j++;
            cloud_cluster.reset(new pcl::PointCloud<pcl::PointXYZ>);
        }
    }

    total_cloud_cluster->width = total_cloud_cluster->size ();
    total_cloud_cluster->height = 1;
    total_cloud_cluster->is_dense = true;

    // Generate map from relative position data
    GenerateMap();

    // Coonvert PCL type to sensor_msgs/PointCloud2 type
    // pcl::toROSMsg(*cloud_filtered, msg_cloud);
    pcl::toROSMsg(*total_cloud_line, msg_cloud);
    // pcl::toROSMsg(*total_cloud_cluster, msg_cloud);
    pcl::toROSMsg(*center_points, msg_obstacle);

    // Free memory
    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    total_cloud_line.reset(new pcl::PointCloud<pcl::PointXYZ>);
    total_cloud_cluster.reset(new pcl::PointCloud<pcl::PointXYZ>);
    center_points.reset(new pcl::PointCloud<pcl::PointXYZ>);

}

void red_ball_cb(core_msgs::ball_position ball_pose) {
    for (int i=0; i<6; i++) {
        if (i < ball_pose.size) {
            relative_position.red_balls[i].x = ball_pose.img_x[i];
            relative_position.red_balls[i].y = ball_pose.img_y[i];
        }
        else {
            relative_position.red_balls[i].x = -100; // Set nonsence value to distinguish later
            relative_position.red_balls[i].y = -100; // Set nonsence value to distinguish later
        }
    }
}

void green_ball_cb(core_msgs::ball_position ball_pose) {
    if (ball_pose.size != 0) {
        relative_position.green_ball.x = ball_pose.img_x[0];
        relative_position.green_ball.y = ball_pose.img_y[0];
    }
}

void time_cb(const std_msgs::Float64ConstPtr &msg){
    timestamp_current = msg->data;
    // cout << "time : " << timestamp_current << endl;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "localization");
    ros::NodeHandle nh;

    ros::Subscriber sub_time = nh.subscribe<std_msgs::Float64>("/simulTime", 1, time_cb);
    ros::Subscriber sub_lidar = nh.subscribe("/scan", 1, lidar_cb);
    ros::Subscriber sub_red_ball = nh.subscribe("/red_position",1,red_ball_cb);
    ros::Subscriber sub_green_ball = nh.subscribe("/green_position",1,green_ball_cb);

    ros::Publisher pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("/map/point_cloud/wall", 1);
    ros::Publisher pub_obstacle_pcl = nh.advertise<sensor_msgs::PointCloud2>("/map/point_cloud/obstacle", 1);

    ros::Publisher pub_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    ros::Publisher pub_robot_geometry = nh.advertise<localization::robot_position>("/map/data/robot",1);
    ros::Publisher pub_obstacle_data = nh.advertise<localization::multi_position>("/map/data/obstacle",1);
    ros::Publisher pub_rb_data = nh.advertise<localization::multi_position>("/map/data/red_ball",1);
    ros::Publisher pub_gb_data = nh.advertise<localization::multi_position>("/map/data/green_ball",1);

    ros::Publisher pub_sys_cmd = nh.advertise<std_msgs::String>("/sys_cmd",1000);

    localization::robot_position msg_robot;
    
    init_vel();

    ros::Rate loop_rate(5);
    while(ros::ok()){
        ros::spinOnce();

        ros::Time current_time = ros::Time::now();

        msg_cloud.header.frame_id = "base_scan"; 
        msg_cloud.header.stamp = current_time;
        pub_cloud.publish(msg_cloud);

        msg_obstacle.header.frame_id = "base_scan"; 
        msg_obstacle.header.stamp = current_time;  
        pub_obstacle_pcl.publish(msg_obstacle);

        msg_robot.header.stamp = current_time;
        msg_robot.x = robot_geometry.pose.x;
        msg_robot.y = robot_geometry.pose.y;
        msg_robot.angle = robot_geometry.angle;
        pub_robot_geometry.publish(msg_robot);

        obstacle_data.header.stamp = current_time;
        pub_obstacle_data.publish(obstacle_data);

        red_balls_data.header.stamp = current_time;
        pub_rb_data.publish(red_balls_data);

        green_ball_data.header.stamp = current_time;
        pub_gb_data.publish(green_ball_data);


        if (mode != 4) {
            pub_vel.publish(vel_msg);
        }

        std::stringstream ss;
        std_msgs::String sys_msg;
        if (mode == 4) {
            ss << "autodriving";
        }
        else {
            ss << "wait";
        }
        sys_msg.data = ss.str();
        pub_sys_cmd.publish(sys_msg);

        loop_rate.sleep();
    }

    return 0;
}
