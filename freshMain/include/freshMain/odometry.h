#ifndef _ODOMETRY_H_
#define _ODOMETRY_H_

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#define PI 3.1415926
#define wheelRadius 0.0498      //m

class Odometry{
private:
    ros::Time current_time, last_time;
    double dt;
public:
    Odometry(double _x, double _y, double _theta);
    double x, y, theta;
    void update(const geometry_msgs::Twist::ConstPtr& ins_vel);
};

#endif