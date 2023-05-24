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
	double coff = 1/ cos(2 * theta);
	if(coor == 'x')
		return coff * (Vy_world * cos(theta) - Vx_world * sin(theta));
	else
		return coff * (Vx_world * cos(theta) - Vy_world * sin(theta));
}
// double vel_Car2World(char coor, double Vx, double Vy){
// 	if(coor == 'x')
// 		return Vx * sin( odom.theta ) + Vy * cos( odom.theta );
// 	else
// 		return Vx * cos( odom.theta ) + Vy * sin( odom.theta );
// }