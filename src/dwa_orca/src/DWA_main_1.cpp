#include "ros/ros.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "KinematicModel.h"
#include "DWA.h"
#include <vector>

// 定义全局变量来存储模型状态信息
gazebo_msgs::ModelStates model_states;
geometry_msgs::Pose robot_initial_pose;
geometry_msgs::Pose current_pose;
geometry_msgs::Pose target_pose; // 目标位置
std::vector<geometry_msgs::Pose> obstacle_poses;
bool received_model_states = false;
std::string specific_model_name; // 特定模型名称
// 回调函数，处理特定模型的信息，并将其他模型信息作为障碍物
void specificModelStateCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
  int specific_model_index = -1;
  // 清空障碍物信息
  obstacle_poses.clear();
  // 遍历接收到的模型状态消息中的所有模型
  for (size_t i = 0; i < msg->name.size(); i++)
  {
    if (msg->name[i] == specific_model_name)
    {
      specific_model_index = i;
    }
    else if (msg->name[i] != "ground_plane") // 排除特定模型和 groud_plan

    {
      // 如果不是特定模型，将其视为障碍物，将其位姿信息添加到障碍物信息中
      obstacle_poses.push_back(msg->pose[i]);
    }
  }

  // 如果找到特定模型，你可以在这里进行特定模型的信息处理
  if (specific_model_index != -1)
  {
    // 获取特定模型的位姿信息
    geometry_msgs::Pose specific_model_pose = msg->pose[specific_model_index];
    current_pose = specific_model_pose;
    // 标记为已接收到模型状态信息
    received_model_states = true;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "DWA_main_1");
  ros::NodeHandle nh("~");
  // 从参数服务器获取特定模型名称
  nh.param<std::string>("specific_model_name", specific_model_name, "model2.5");
  // 订阅Gazebo中的模型状态信息
  ros::Subscriber modelStateSub = nh.subscribe("/gazebo/model_states", 1, specificModelStateCallback);
  // 创建发布器
  ros::Publisher modelStatePub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);
  geometry_msgs::Pose final_pose;
  geometry_msgs::Twist twist;
  double time, num, max_angular_speed, max_linear_speed;
  double target_x, target_y;
  double target_orientation_z, target_orientation_w;
  // 获取参数
  nh.param("time", time, 0.05);
  nh.param("num", num, 10.0);
  nh.param("max_angular_speed", max_angular_speed, 3.0);
  nh.param("max_linear_speed", max_linear_speed, 3.0);
  nh.param("target_x", target_x, 0.0);
  nh.param("target_y", target_y, 0.0);
  // nh.param("target_orientation_z", target_orientation_z, sin(M_PI/4.0));
  // nh.param("target_orientation_w", target_orientation_w, cos(M_PI/4.0));
  // 初始化目标位置
  target_pose.position.x = target_x;
  target_pose.position.y = target_y;
  target_pose.orientation.z = sin(M_PI / 4.0);
  target_pose.orientation.w = cos(M_PI / 4.0);
  ros::Rate loop_rate(50);
  while (ros::ok())
  {
    if (!received_model_states)
    {
      // ROS_INFO("Waiting for model states...");
      ros::spinOnce();
      continue;
    }
    ROS_INFO("Specific Model Name: %s", specific_model_name.c_str());
    // 创建DWAPlanner对象
    DWAPlanner planner(target_pose, obstacle_poses, max_linear_speed, max_angular_speed, time, num, current_pose);
    // 查找最佳速度，final_pose，最佳分数
    const geometry_msgs::Twist &best_twist = planner.FindBestTwist(current_pose);

    // 使用运动学模型更新机器人的位姿
    twist = best_twist;
    final_pose = planner.GetFinalPose();
    double best_score = planner.GetBestScore();
    // 创建运动学模型对象
    KinematicModel kinematicModel(current_pose, twist);
    geometry_msgs::Pose newpose = kinematicModel.calculateNewPosition(time);
    // 发布机器人的位姿
    gazebo_msgs::ModelState modelState;
    modelState.model_name = specific_model_name;
    modelState.pose = newpose;
    modelStatePub.publish(modelState);
    // 输出最佳速度、分数、final_pose和迭代序号
    // std::cout << " - Best Twist: linear.x = " << twist.linear.x << ", angular.z = " << twist.angular.z
    //           << std::endl;
    //  std::cout << " - Best Score: " << best_score << std::endl;
    // std::cout << " - Best Final Pose: x = " << final_pose.position.x << ", y = " << final_pose.position.y <<
    // std::endl;
    ROS_INFO(" - Best Twist: linear.x = %f, angular.z = %f", twist.linear.x, twist.angular.z);
    ROS_INFO(" - Best Score: %f", best_score);
    ROS_INFO(" - Best Final Pose: x = %f, y = %f", final_pose.position.x, final_pose.position.y);
    current_pose = newpose; // 更新位姿
    // 检查是否到达目标点
    double distance_to_target = sqrt(pow(target_pose.position.x - current_pose.position.x, 2) +
                                     pow(target_pose.position.y - current_pose.position.y, 2));
    if (distance_to_target < 1)
    {
      ROS_INFO("Reached the goal");
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
