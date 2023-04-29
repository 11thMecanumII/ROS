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

#define allowance 10e-3
#define NSS 30 //num_of_SoftStart

void clearScreen();
class Mecanum{
private:
    // double maxSpeed = 1.5645;       //m/s
    double maxSpeed = 0.6;       //m/s
    double limit;
    double X, Y, W;     //x-speed, y-speed, w:angular speed
    double GS[4];      //goalSpeed1 2 3 4 
    double scanRatio;
public:
    int softStart;
    double maxGS;
    Odometry odometry;
    Mecanum(){
        softStart = 0;
        odometry.x = 0;
        odometry.y = 0;
        odometry.theta = 0;
    }
    bool if_reach = false;
    geometry_msgs::Twist goTo(double des_x, double des_y, double des_theta, double speed_Kp){
        geometry_msgs::Twist speed;
        double diff_x, diff_y, diff_theta;
        
        diff_x = des_x - odometry.x;    
        diff_y = des_y - odometry.y;    
        diff_theta = (des_theta/180*PI) - odometry.theta;
        while(std::abs(diff_theta) > PI)   diff_theta = (diff_theta > 0)?diff_theta - 2*PI:diff_theta + 2*PI;
        
        X = (std::abs(diff_x) > allowance)?std::abs(speed_Kp) * diff_x:0;
        Y = (std::abs(diff_y) > allowance)?std::abs(speed_Kp) * diff_y:0;
        W = (std::abs(diff_theta) > allowance)?std::abs(speed_Kp) * diff_theta:0;

        if(X == 0 && Y == 0 && W == 0){
            if_reach = true;
        }        

        limit = (softStart < NSS)?softStart/NSS*maxSpeed:maxSpeed;
        GS[0] = Y + X + W*0.152767;
        GS[1] = Y + X - W*0.152844;
        GS[2] = Y - X + W*0.152998;
        GS[3] = Y - X - W*0.154852;
        for(int i = 0; i < 4; i ++){
            if(std::abs(GS[i]) > std::abs(maxGS)){
                maxGS = GS[i];
            }
        }
        if(std::abs(maxGS) > limit){
            scanRatio = limit / std::abs(maxGS);
            X *= scanRatio;
            Y *= scanRatio;
            W *= scanRatio;
        }
        speed.linear.x = X;
        speed.linear.y = Y;
        speed.angular.z = W;

        std::cout<<std::fixed<<std::setprecision(0);
        std::cout<<softStart<<"\t";
        
        std::cout<<std::fixed<<std::setprecision(4);
        std::cout<<"limit:"<<limit<<"\t";
        std::cout<<"des("<<des_x<<","<<des_y<<","<<des_theta<<")\t";
        std::cout<<"odo("<<odometry.x<<","<<odometry.y<<","<<odometry.theta*180/PI<<")\t";
        std::cout<<"vel("<<speed.linear.x<<","<<speed.linear.y<<","<<speed.angular.z<<")\n";

        return speed;
    }
    void initPosition(double x, double y, double theta){
        odometry.x = x;
        odometry.y = y;
        odometry.theta = theta/180*PI;
    }
};
int readPath(double* des_x_Ptr, double* des_y_Ptr, double* des_theta_Ptr, size_t& current_index);
#endif