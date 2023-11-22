#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/ModelState.h"
#include "std_msgs/String.h"
#include "ModelSubPub.h"
#include "Neighbor.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pub_sub");
    ros::NodeHandle nh("~");
    // 从参数服务器中获取相应的参数值
    std::string targetModelName;
    nh.getParam("target_model_name", targetModelName);
    double twist_linear_x, twist_angular_z, maxSpeed_, neighborDistance_, timeHorizon_, radius_, time;
    nh.getParam("target_model/twist_linear_x", twist_linear_x);
    nh.getParam("target_model/twist_angular_z", twist_angular_z);
    nh.getParam("max_speed", maxSpeed_);
    nh.getParam("neighbor_distance", neighborDistance_);
    nh.getParam("time_horizon", timeHorizon_);
    nh.getParam("radius_", radius_);
    nh.getParam("time", time);
    geometry_msgs::Pose goal_pose;
    double goal_pose_x, goal_pose_y, goal_pose_z;
    nh.getParam("goal_pose_x", goal_pose_x);
    nh.getParam("goal_pose_y", goal_pose_y);
    nh.getParam("goal_pose_z", goal_pose_z);
    goal_pose.position.x = goal_pose_x;
    goal_pose.position.y = goal_pose_y;
    goal_pose.position.z = goal_pose_z;
    //  创建 ModelSubPub 实例并传入参数
    gazebo_msgs::ModelState target_model_state;
    target_model_state.twist.linear.x = twist_linear_x;
    target_model_state.twist.angular.z = twist_angular_z;
    RVO::ModelSubPub modelSubPub(targetModelName, time, target_model_state, goal_pose,
                                 maxSpeed_, neighborDistance_, timeHorizon_, radius_);
    ros::Rate rate(100);
    while (ros::ok())
    {
        ROS_INFO("Target model name: %s", targetModelName.c_str());
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
