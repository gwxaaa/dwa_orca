#ifndef KINEMATIC_MODEL_H
#define KINEMATIC_MODEL_H

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
class KinematicModel {
public:
    KinematicModel(const geometry_msgs::Pose& pose, const geometry_msgs::Twist& twist);
    geometry_msgs::Pose calculateNewPosition(double time);
    double getCenterX() const; 
    double getCenterY() const;
    double getYaw() const;

private: 
  geometry_msgs::Pose pose;
  geometry_msgs::Twist twist;
  geometry_msgs::Pose result;
  double center_x;
  double center_y;
  

};

#endif // KINEMATIC_MODEL_H