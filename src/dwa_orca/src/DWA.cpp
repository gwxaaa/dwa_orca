#include "DWA.h"
#include <cmath>
#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/ModelState.h>
#include <KinematicModel.h>
DWAPlanner::DWAPlanner(const geometry_msgs::Pose& target_pose, const std::vector<geometry_msgs::Pose>& obstacles,
                       double max_linear_speed, double max_angular_speed, double time, double num,
                       const geometry_msgs::Pose& current_pose)
  : target_pose(target_pose)
  , obstacles(obstacles)
  , max_distance(1.0)
  , some_threshold(some_threshold)
  , max_linear_speed(max_linear_speed)
  , max_angular_speed(max_angular_speed)
  , time(time)
  , num(num)
{
}
DWAPlanner::~DWAPlanner()
{
}
const geometry_msgs::Twist& DWAPlanner::FindBestTwist(const geometry_msgs::Pose& current_pose)
{
  // 初始化最佳速度和最佳分数
  best_twist = current_twist;
  double best_score = -std::numeric_limits<double>::infinity();
  std::vector<geometry_msgs::Twist> twist_combinations = GenerateTwists();
  double max_distance = FindMaxDistance(twist_combinations, current_pose);
  // std::cout << "Max Distance: " << max_distance << std::endl;
  // 创建一个临时变量来存储最佳速度
  geometry_msgs::Twist best_temp_twist = best_twist;
  geometry_msgs::Pose best_final_pose = final_pose;
  double best_final_score = -0.1;
  // 循环评估速度组合
  for (const auto& current_twist : twist_combinations)
  {
    // 预测最终姿态
    final_pose = PredictPose(current_pose, current_twist, time);
    // 检查是否接近目标点
    double distance_to_target = CalculateDistance(final_pose);
    // 检查碰撞
    double collision = CalculateCollision(final_pose);
    // 如果当前组合的分数更高，更新最佳分数和速度角速度
    double score = CalculateScore(current_twist);
    // std::cout << "Score: " << score << std::endl;
    // std::cout << "Score1: " << best_final_score << std::endl;
    // std::cout << "twist: " << current_twist<< std::endl;
    //    std::cout << " - Best Final Pose: x = " << final_pose.position.x << ", y = " << final_pose.position.y <<
    // std::endl;
    if (score > best_final_score)
    {
      best_final_score = score;
      best_temp_twist = current_twist;
      best_final_pose = final_pose;
    }
  }
  // 更新最佳速度
  best_twist = best_temp_twist;
  final_pose = best_final_pose;
  return best_twist;
}

std::vector<geometry_msgs::Twist> DWAPlanner::GenerateTwists()
{
  std::vector<geometry_msgs::Twist> twist_combinations;  // 存储多组速度角速度组合
  double linear_speed_increment = (2 * max_linear_speed) / static_cast<double>(num - 1);
  double angular_speed_increment = (2 * max_angular_speed) / static_cast<double>(num - 1);
  for (int i = 0; i < num; i++)
  {
    for (int j = 0; j < num; j++)
    {
      // 创建速度组合
      geometry_msgs::Twist current_twist;
      current_twist.linear.x = -max_linear_speed + i * linear_speed_increment;
      current_twist.angular.z = -max_angular_speed + j * angular_speed_increment;
      twist_combinations.push_back(current_twist);  // 存储当前组合
    }
  }

  return twist_combinations;
}

geometry_msgs::Pose DWAPlanner::PredictPose(const geometry_msgs::Pose& current_pose, const geometry_msgs::Twist& twist,
                                            double time)
{
  KinematicModel kinematic_model(current_pose, twist);
  geometry_msgs::Pose final_pose = kinematic_model.calculateNewPosition(time);
  return final_pose;
}

// double DWAPlanner::CalculateCollision(const geometry_msgs::Pose& final_pose)
// {
//   double collision_distance_threshold = 1.7;  // 设置碰撞距离阈值，
//   for (const geometry_msgs::Pose& obstacle : obstacles)
//   {
//     // 计算机器人和障碍物之间的距离
//     double dx = final_pose.position.x - obstacle.position.x;
//     double dy = final_pose.position.y - obstacle.position.y;

//     double distance1 = std::sqrt(dx * dx + dy * dy);

//     double dx1 = current_pose.position.x - obstacle.position.x;
//     double dy1 = current_pose.position.y - obstacle.position.y;
//       double distance2 = std::sqrt(dx1 * dx1 + dy1 * dy1);
//   if (distance1 <= collision_distance_threshold  )
//     {
//       double score = 1-distance1 / collision_distance_threshold / 2; 
//       return score;
//     }
//     // else
//     // {
//     //   double score=1.0;
//     //   return score;
//     // }

//   }
//   // 返回综合得分
//   return 1;
// }

double DWAPlanner::CalculateCollision(const geometry_msgs::Pose& final_pose)
{
  double min_collision_distance = 0.4;      // 最小碰撞距离
  double avoidance_distance = 1.7;         // 避障范围起始距离
  double collision_distance_threshold = 1.7; // 碰撞阈值

  double distance_to_obstacle = std::numeric_limits<double>::max();

  for (const geometry_msgs::Pose& obstacle : obstacles)
  {
    double dx = final_pose.position.x - obstacle.position.x;
    double dy = final_pose.position.y - obstacle.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance < distance_to_obstacle)
    {
      distance_to_obstacle = distance;
    }
  }

  if (distance_to_obstacle < min_collision_distance)
  {
    // 如果距离障碍物小于最小碰撞距离，直接返回0分表示碰撞
    return 0.0;
  }
  else if (distance_to_obstacle <= avoidance_distance)
  {
    // 如果距离障碍物在避障范围内，根据距离插值计算分数
    double score = 1.0 - (distance_to_obstacle - min_collision_distance) /
                           (avoidance_distance - min_collision_distance);
    return score;
  }
  else if (distance_to_obstacle <= collision_distance_threshold)
  {
    // 如果距离障碍物在碰撞阈值内，给予一定的分数
    double score = 0.5; // 你可以根据需要调整这个分数
    return score;
  }
  else
  {
    // 如果没有遇到障碍物或者距离障碍物超出碰撞阈值，返回最高分数
    return 1.0;
  }
}

double DWAPlanner::FindMaxDistance(const std::vector<geometry_msgs::Twist>& twist_vector,
                                   const geometry_msgs::Pose& current_pose)
{
  double max_distance = 1.0;
  // 遍历每组速度角速度，计算对应的距离
  for (const auto& twist : twist_vector)
  {
    geometry_msgs::Pose final_pose = PredictPose(current_pose, twist, time);
    double dx = final_pose.position.x - target_pose.position.x;
    double dy = final_pose.position.y - target_pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    // 更新最大距离
    if (distance > max_distance)
    {
      max_distance = distance;
    }
  }
  return max_distance;
}

double DWAPlanner::CalculateDistance(const geometry_msgs::Pose& final_pose)
{
  // 计算位置之间的距离
  double dx = final_pose.position.x - target_pose.position.x;
  double dy = final_pose.position.y - target_pose.position.y;
  double distance_position = std::sqrt(dx * dx + dy * dy);
  // 计算朝向之间的角度差异
  double target_yaw = atan2(2.0 * (target_pose.orientation.z * target_pose.orientation.w +
                                   target_pose.orientation.x * target_pose.orientation.y),
                            1.0 - 2.0 * (target_pose.orientation.y * target_pose.orientation.y +
                                         target_pose.orientation.z * target_pose.orientation.z));
  double final_yaw = atan2(
      2.0 * (final_pose.orientation.z * final_pose.orientation.w + final_pose.orientation.x * final_pose.orientation.y),
      1.0 - 2.0 * (final_pose.orientation.y * final_pose.orientation.y +
                   final_pose.orientation.z * final_pose.orientation.z));
  double yaw_difference = std::abs(target_yaw - final_yaw);
  // 将角度差归一化为 [0, 1]
  double normalized_angle_distance = yaw_difference / 2 * M_PI;
  //将距离归一化为【0，1】
  //最大距离引入
  double normalized_distance_distance = distance_position / (max_distance * 1);
  double distance = normalized_distance_distance;
  return distance;
}

double DWAPlanner::CalculateScore(const geometry_msgs::Twist& twist)
{
  // 计算距离目标的距离
  double distance = CalculateDistance(final_pose);
  // 计算碰撞惩罚
  double collision = CalculateCollision(final_pose);

  // if (collision<1)
  // {
  //   distance=-distance;
  // }


  // 设置权重
  double distance_weight = -0.1;
  double collision_weight = 100000;
  // 计算距离分数和碰撞分数
  double distance_score = distance_weight * distance;
  double collision_score = collision_weight * collision;
  // 计算总分数
  double score = collision_score + distance_score;
  // 如果当前分数比最佳分数高，更新最佳分数和对应的 twist 值
  if (score > best_final_score)
  {
    best_final_score = score;
    best_twist = twist;
  }
  

  return score;
}
