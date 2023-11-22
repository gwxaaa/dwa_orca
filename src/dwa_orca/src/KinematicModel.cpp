#include "KinematicModel.h"
#include <cmath>

#include<geometry_msgs/Quaternion.h>


KinematicModel::KinematicModel(const geometry_msgs::Pose& pose, const geometry_msgs::Twist& twist) { 
    this->pose = pose; 
    this->twist = twist;
}

geometry_msgs::Pose KinematicModel::calculateNewPosition(double time) {

    double radius;
    double Px;
    double Py;
    double center_x;
    double center_y;
    double  initial_yaw=atan2(2.0*(pose.orientation.w*pose.orientation.z+pose.orientation.x*pose.orientation.y),
                         1.0-2.0*(pose.orientation.y*pose.orientation.y+pose.orientation.z*pose.orientation.z));

    double initial_angle=initial_yaw;
    geometry_msgs::Pose result=this->pose;

    double theta = this->twist.angular.z * time;
    if (std::abs(this->twist.angular.z) < 0.001) {
        // 直线运动的位置计算
        this->result.position.x = pose.position.x + twist.linear.x * cos(initial_angle) * time;
        this->result.position.y = pose.position.y + twist.linear.x * sin(initial_angle) * time;
        this->result.orientation.x = pose.orientation.x ;
        this->result.orientation.y = pose.orientation.y ;
        this->result.orientation.z = pose.orientation.z ;
        this->result.orientation.w = pose.orientation.w ;

        } 
        else {
            // 计算圆弧半径
            double radius =twist.linear.x /  twist.angular.z ;
            center_x =  pose.position.x - radius * sin(initial_angle) ;
            center_y =  pose.position.y + radius * cos(initial_angle );

             this->center_x = center_x; 
             this->center_y = center_y;
 
            
        // 计算旋转后的四元数
             double new_yaw = twist.angular.z * time;
             double half_new_yaw = new_yaw * 0.5;
             double sin_half_new_yaw = sin(half_new_yaw);
             double cos_half_new_yaw = cos(half_new_yaw);
             double sin_initial_yaw = sin(initial_yaw * 0.5);
             double cos_initial_yaw = cos(initial_yaw * 0.5);

            this->result.position.x = center_x + (pose.position.x- center_x) * cos(theta)-(pose.position.y- center_y)* sin(theta);
            this->result.position.y = center_y + (pose.position.x- center_x) * sin(theta)+(pose.position.y- center_y)* cos(theta);

            this->result.orientation.x = pose.orientation.x * cos_half_new_yaw + pose.orientation.y * sin_half_new_yaw;
            this->result.orientation.y = pose.orientation.y * cos_half_new_yaw - pose.orientation.x * sin_half_new_yaw;
            this->result.orientation.z = pose.orientation.z * cos_half_new_yaw + pose.orientation.w * sin_half_new_yaw;
            this->result.orientation.w = pose.orientation.w * cos_half_new_yaw - pose.orientation.z * sin_half_new_yaw;
          
    }   

     return this->result;
}

                 
double KinematicModel::getCenterX() const { 
    return center_x; 
    }
     double KinematicModel::getCenterY() const {
         return center_y;
          }