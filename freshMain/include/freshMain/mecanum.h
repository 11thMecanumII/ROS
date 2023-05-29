#ifndef _MECANUM_H_ 
#define _MECANUM_H_

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "odometry.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <iomanip>
#include <cstdlib>

// #define filePath "/home/ditrobotics/catkin_ws/src/freshMain/params/11project.yaml"
#define filePath "/home/ditrobotics/catkin_ws/src/freshMain/params/11project copy.yaml"
// #define filePath "/home/ditrobotics/catkin_ws/src/freshMain/params/test123.yaml"

#define allowance 10e-4
#define NSS 15 //num_of_SoftStart
#define maxSpeed_true 1.5645    //m/s
#define maxSpeed_choose 0.5
// #define centerDistance1 0.152767
// #define centerDistance2 0.152844
// #define centerDistance3 0.152998
// #define centerDistance4 0.154852

void clearScreen();
class Mecanum{
private:
    const double maxSpeed = maxSpeed_choose;
    double limit;
    double X, Y, W;     //x-speed, y-speed, w:angular speed
    double GS[4];      //goalSpeed1 2 3 4 
    double scanRatio;
public:
    Mecanum(double x, double y, double theta);
    int softStart;
    double maxGS;
    Odometry odometry;
    bool if_reach;
    geometry_msgs::Twist goTo(double des_x, double des_y, double des_theta, double speed_Kp);
    void initPosition(double x, double y, double theta);
};
int readPath(double* des_x_Ptr, double* des_y_Ptr, double* des_theta_Ptr, size_t& current_index);
#endif