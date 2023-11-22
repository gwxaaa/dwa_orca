#include "ros/ros.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "KinematicModel.h"
#include "DWA.h"
#include <vector>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "DWA_main");
  ros::NodeHandle nh;

  int iteration;
  double time = M_PI / 8.0;
  double num = 21;
  double max_angular_speed = 3.0;
  double max_linear_speed = 3.0;
  geometry_msgs::Twist twist;
  geometry_msgs::Pose final_pose;

  // 定义当前和目标位置
  geometry_msgs::Pose current_pose;
  current_pose.position.x = 0.0;
  current_pose.position.y = 0.0;
  current_pose.orientation.z = 0.0;
  current_pose.orientation.w = 1.0;

  geometry_msgs::Pose target_pose;
  target_pose.position.x = 20.0;
  target_pose.position.y = 10.0;
  target_pose.orientation.z = 5.0;
  target_pose.orientation.w = 1.0;

  // 定义一组障碍物
  std::vector<geometry_msgs::Pose> obstacles;
  geometry_msgs::Pose obstacle1;
  geometry_msgs::Pose obstacle2;
  obstacle1.position.x = -30.0;
  obstacle1.position.y = 0.1;
  obstacle1.orientation.z = 0.0;
  obstacle1.orientation.w = 1.0;
  obstacles.push_back(obstacle1);
  obstacle2.position.x = -20.0;
  obstacle2.position.y = 1.0;
  obstacle2.orientation.z = 0.0;
  obstacle2.orientation.w = 1.0;
  obstacles.push_back(obstacle2);

  bool reachedGoal = false;
  int maxIterations = 30;  // 最大迭代次数
  int currentIteration = 0;

  while (!reachedGoal && currentIteration < maxIterations)
  {
    // 创建DWAPlanner·对象
    DWAPlanner planner(target_pose, obstacles, max_linear_speed, max_angular_speed, time, num);
    // 查找最佳速度，final_pose，最佳分数
    const geometry_msgs::Twist& best_twist = planner.FindBestTwist(current_pose);

    final_pose = planner.GetFinalPose();
    double best_score = planner.GetBestScore();
    // 输出最佳速度、分数、final_pose和迭代序号
    std::cout << "Iteration " << currentIteration << " - Best Twist: linear.x = " << best_twist.linear.x
              << ", angular.z = " << best_twist.angular.z << std::endl;
    std::cout << "Iteration " << currentIteration << " - Best Score: " << best_score << std::endl;
    std::cout << "Iteration " << currentIteration << " - Best Final Pose: x = " << final_pose.position.x
              << ", y = " << final_pose.position.y << std::endl;
    current_pose = final_pose;  // 更新位姿
    twist = best_twist;
    // 检查是否到达目标点
    double distance_to_target = sqrt(pow(target_pose.position.x - current_pose.position.x, 2) +
                                     pow(target_pose.position.y - current_pose.position.y, 2));
    if (distance_to_target < 0.1)
    {
      reachedGoal = true;
      std::cout << "success" << std::endl;
    }
    currentIteration++;
  }
  return 0;
}
