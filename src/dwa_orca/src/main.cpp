#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "gazebo_msgs/ModelStates.h"
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include "KinematicModel.h" // 引入头文

int main(int argc, char **argv) {
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    // 创建一个测试用的 geometry_msgs::Pose 和 geometry_msgs::Twist 对象
    geometry_msgs::Pose pose;
    pose.position.x = 1.0;
    pose.position.y = 1.0;
    //
    double radian= M_PI/4.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = sin(radian/2);
    pose.orientation.w = cos(radian/2); 
    geometry_msgs::Twist twist;
    twist.linear.x = 1.0; // 设置线速度
    twist.angular.z =1.00; // 设置角速度   
    // 创建 KinematicModel 对象
    KinematicModel kinematicModel(pose, twist);
    // 计算新位置
    double time = M_PI/2.0  ; // 
    geometry_msgs::Pose result = kinematicModel.calculateNewPosition( time);
    double center_x = kinematicModel.getCenterX();
    double center_y = kinematicModel.getCenterY();
    ROS_INFO("New Position: x = %f, y = %f", result.position.x, result.position.y);
    //ROS_INFO("New orientation: x = %f, y = %f,z=%f,w=%f", result.orientation.x, result.orientation.y,result.orientation.z,result.orientation.w);
    //result=pose;
 
    double yaw = atan2(    2.0*(result.orientation.w*result.orientation.z+result.orientation.x*result.orientation.y),
                       1.0-2.0*(result.orientation.y*result.orientation.y+result.orientation.z*result.orientation.z));
   //四元数转化为欧拉角，四元数中的信息是欧拉角的一半，然后atan2的取值是-pi-----pi，所以超过180度， 就会变成负数。
   //

    ROS_INFO("Yaw:%f degrees",yaw);

    return 0;
}