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
double Odometry::vel_World2Car(char coor, double Vx_world, double Vy_world){
	if(coor == 'x')
        return Vx_world * cos(theta) - Vy_world * sin(theta);
	else
		return Vx_world * cos(theta) + Vy_world * sin(theta);
}
double Odometry::vel_Car2World(char coor, double Vx, double Vy){
	if(coor == 'x')
		return Vx * sin(theta) + Vy * cos(theta);
	else
		return Vy * sin(theta) - Vx * cos(theta);
}