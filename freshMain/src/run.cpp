#include "cmath"
#include "iostream"
#include "sys/time.h"

#include "odometry.h"
#include "mecanum.h"

#define numOfPoints 12

double speed_Kp = 10, des_x, des_y, des_theta;
double des_x_last = -1, des_y_last = -1, des_theta_last = -1;
size_t current_index = 0;

Odometry odometry;
Mecanum mecanum;


void Callback(const geometry_msgs::Twist::ConstPtr& ins_vel){
    mecanum.odometry.update(ins_vel);
    mecanum.softStart++;     //to sure it can do whole softStart;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "run");
    ros::NodeHandle nh;
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Subscriber pose_sub = nh.subscribe("/ins_vel",1,Callback);
    mecanum.initPosition(0,0,90);
    while(ros::ok()){
        if(readPath(&des_x, &des_y, &des_theta, current_index))     break;
        std::cout<<current_index<<" : \t";
        std::cout<<"("<<des_x<<"\t"<<des_y<<"\t"<<des_theta<<")\n";
        ros::Duration(1.0).sleep(); // Sleep for 1 second
        if(current_index > numOfPoints)  break;
        if(des_x_last != des_x || des_y_last != des_y || des_theta_last != des_theta){
            mecanum.softStart = 0;
            while(!mecanum.if_reach && ros::ok()){
                ros::spinOnce();
                vel_pub.publish( mecanum.goTo(des_x, des_y, des_theta, speed_Kp) );
                mecanum.maxGS = 0;
                ros::Duration(0.02).sleep();    //50Hz
            }
            mecanum.if_reach = false;
            std::cout<<"\n\t\tarrive the ("<<current_index<<" th) destanation!\n\n";
            ros::Duration(1.0).sleep(); // Sleep for 1 second
            clearScreen();
        }
        des_x_last = des_x;
        des_y_last = des_y;
        des_theta_last = des_theta;
    }
    return 0;
}