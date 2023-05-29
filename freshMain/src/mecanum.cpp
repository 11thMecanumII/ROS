#include "mecanum.h"

void clearScreen(){
#ifdef _WIN32
    std::system("cls");
#else
    std::system("clear");
#endif
}
Mecanum::Mecanum(double x, double y, double theta):
    odometry(x,y,theta), softStart(0), if_reach(false){};
geometry_msgs::Twist Mecanum::goTo(double des_x, double des_y, double des_theta, double speed_Kp){
    geometry_msgs::Twist speed;
    double diff_x, diff_y, diff_theta;
    
    diff_x = des_x - odometry.x;    
    diff_y = des_y - odometry.y;    
    diff_theta = (des_theta/180*PI) - odometry.theta;
    while(std::fabs(diff_theta) > PI)   diff_theta = (diff_theta > 0)?diff_theta - 2*PI:diff_theta + 2*PI;
    
    X = (std::fabs(diff_x) > allowance)?std::fabs(speed_Kp) * diff_x:0;
    Y = (std::fabs(diff_y) > allowance)?std::fabs(speed_Kp) * diff_y:0;
    W = (std::fabs(diff_theta) > allowance)?std::fabs(speed_Kp) * diff_theta:0;

    if(X == 0 && Y == 0 && W == 0){
        if_reach = true;
    }        

    limit = (softStart < NSS)? (double)((double)softStart*maxSpeed)/NSS: maxSpeed;
    double carX = odometry.vel_World2Car('x', X, Y);
    double carY = odometry.vel_World2Car('y', X, Y);
    double carW = W;
    GS[0] = carY + carX + W*0.5*(carWidth + carLength);
    GS[1] = carY + carX - W*0.5*(carWidth + carLength);
    GS[2] = carY - carX + W*0.5*(carWidth + carLength);
    GS[3] = carY - carX - W*0.5*(carWidth + carLength);
    for(int i = 0; i < 4; i ++){
        if(std::fabs(GS[i]) > std::fabs(maxGS)){
            maxGS = GS[i];
        }
    }
    if(std::fabs(maxGS) > limit){
        scanRatio = (double) limit / std::fabs(maxGS);
    }
    else{
        scanRatio = 1;
    }
    X *= scanRatio;
    Y *= scanRatio;
    W *= scanRatio;
    speed.linear.x = X;
    speed.linear.y = Y;
    speed.angular.z = W;

    std::cout<<std::fixed<<std::setprecision(0);
    std::cout<<softStart<<"\t";
    std::cout<<std::fixed<<std::setprecision(4);
    std::cout<<"scanRatio: "<<scanRatio<<"\t";
    std::cout<<"["<<carX<<" "<<carY<<" "<<carW<<"]\t";
    std::cout<<GS[0]<<" "<<GS[1]<<" "<<GS[2]<<" "<<GS[3]<<"\t";
    std::cout<<"("<<odometry.x<<" "<<odometry.y<<" "<<odometry.theta<<")\n";
    // std::cout<<"limit:"<<limit<<"\t";
    // std::cout<<"des("<<des_x<<","<<des_y<<","<<des_theta<<")\t";
    // std::cout<<"odo("<<odometry.x<<","<<odometry.y<<","<<odometry.theta*180/PI<<")\t";
    // std::cout<<"vel("<<speed.linear.x<<","<<speed.linear.y<<","<<speed.angular.z<<")\n";

    return speed;
}
int readPath(double* des_x_Ptr, double* des_y_Ptr, double* des_theta_Ptr, size_t& current_index){
    std::ifstream file(filePath);
    if (!file.is_open()) {
        std::cerr << "Error opening file!" << std::endl;
        return 1;
    }

    YAML::Node pathConfig = YAML::Load(file);
    if (current_index >= pathConfig.size()) {
        std::cout<<"over\n";
        return 1;
    }

    int i = -1;
    for (auto twistElement : pathConfig) {
        auto twist = twistElement["twist"];
        *des_x_Ptr = twist[0].as<double>();
        *des_y_Ptr = twist[1].as<double>();
        *des_theta_Ptr = twist[2].as<double>();
        if(++i == current_index)  break;
    }
    current_index ++;
    return 0;
}