#include "cmath"
#include "iostream"
#include <sys/time.h>

#include "odometry.h"
#include "mecanum.h"

#define numOfPoints 12

double speed_Kp = 10, des_x, des_y, des_theta;
double des_x_last = -1, des_y_last = -1, des_theta_last = -1;
size_t current_index = 0;

Mecanum mecanum(0, 0, 90);

void Callback(const geometry_msgs::Twist::ConstPtr& ins_vel){
    mecanum.odometry.update(ins_vel);
}

void my_interrupt_handler(int gpio, int level, uint32_t tick) {
    std::cout<<"tick: "<<tick<<"\t";
    std::cout<<"level: "<<level<<"\t";
    std::cout<<"gpio: "<<gpio<<"\t";
    std::cout<<"------------------\n";
    
    vel_pub.publish( mecanum.goTo(des_x, des_y, des_theta, speed_Kp) );
    mecanum.softStart++;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "run");
    ros::NodeHandle nh;
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Subscriber pose_sub = nh.subscribe("/ins_vel",1,Callback);

    // Initialize pigpio
    gpioInitialise();
    
    // Configure the timer to generate interrupts at 50Hz
    // Assume the timer is called "timer0"
    int period_us = 20000; // Period in microseconds
    int frequency_hz = 50; // Frequency in Hz
    int prescaler = 192; // Prescaler value
    int timer_period = (int)(period_us * frequency_hz / 1000000.0 * prescaler);
    gpioSetTimerFunc(0, timer_period, my_interrupt_handler); // Set up the timer

    while(ros::ok()){
        ros::spinOnce();
        if(mecanum.if_reach){
            std::cout<<"\n\t\tarrive the ("<<current_index<<" th) destanation!\n\n";
            mecanum.if_reach = false;
            mecanum.softStart = 0;
            if(readPath(&des_x, &des_y, &des_theta, current_index))     break;
            if(current_index > numOfPoints)  break;
            clearScreen();
            ros::Duration(1.0).sleep();
            std::cout<<current_index<<" : \t";
            std::cout<<"("<<des_x<<"\t"<<des_y<<"\t"<<des_theta<<")\n";
        }
        // if(des_x_last != des_x || des_y_last != des_y || des_theta_last != des_theta){
        //     des_x_last = des_x;
        //     des_y_last = des_y;
        //     des_theta_last = des_theta;
        // }
    }

    return 0;
}