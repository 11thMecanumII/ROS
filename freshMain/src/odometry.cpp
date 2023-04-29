#include "odometry.h"

Odometry::Odometry(double x, double y, double theta):
    x(x), y(y), theta(theta/180*PI){};
void Odometry::update(const geometry_msgs::Twist::ConstPtr& ins_vel){
    current_time = ros::Time::now();
    dt = (current_time - last_time).toSec();
    if(std::abs(dt) > 1)  dt = 0;
    x += ins_vel->linear.x * (dt);
    y += ins_vel->linear.y * (dt);
    theta += ins_vel->angular.z * (dt);
    while(theta > PI)  theta -= 2*PI;        
    while(theta < -PI) theta += 2*PI;
    last_time = current_time;
    return;
}