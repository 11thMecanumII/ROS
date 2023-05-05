#include "mecanum.h"

void clearScreen(){
#ifdef _WIN32
    std::system("cls");
#else
    std::system("clear");
#endif
}
Mecanum::Mecanum(double x, double y, double theta):
    odometry(x,y,theta), softStart(0), if_reach(true){};

geometry_msgs::Twist Mecanum::goTo(double des_x, double des_y, double des_theta, double speed_Kp){
    geometry_msgs::Twist speed;
    double diff_x, diff_y, diff_theta;
    
    diff_x = des_x - odometry.x;    
    diff_y = des_y - odometry.y;    
    diff_theta = (des_theta/180*PI) - odometry.theta;
    while(std::abs(diff_theta) > PI)   diff_theta = (diff_theta > 0)?diff_theta - 2*PI:diff_theta + 2*PI;
    
    X = (std::abs(diff_x) > allowance)?std::abs(speed_Kp) * diff_x:0;
    Y = (std::abs(diff_y) > allowance)?std::abs(speed_Kp) * diff_y:0;
    W = (std::abs(diff_theta) > allowance)?std::abs(speed_Kp) * diff_theta:0;

    limit = (softStart < NSS)?(double)((double)softStart*maxSpeed)/NSS:maxSpeed;
    GS[0] = Y + X + W*centerDistance1;
    GS[1] = Y + X - W*centerDistance2;
    GS[2] = Y - X + W*centerDistance3;
    GS[3] = Y - X - W*centerDistance4;
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

    maxGS = 0;
    if(X == 0 && Y == 0 && W == 0)        if_reach = true;
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