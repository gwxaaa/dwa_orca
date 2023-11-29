#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/ModelState.h"
#include "std_msgs/String.h"
#include "ModelSubPub.h"
#include "Agent.h"
#include "Neighbor.h"
#include <cmath>
#include <KinematicModel.h>
#include "DWA.h"
#include <geometry_msgs/PoseStamped.h>
#include "nav_msgs/Path.h"
namespace RVO
{
  ModelSubPub::ModelSubPub(const std::string &modelName, double time, gazebo_msgs::ModelState target_model_state,
                           geometry_msgs::Pose goal_pose, double maxSpeed_, double neighborDistance_, double timeHorizon_, double radius_, double num,
                           double max_linear_speed, double max_angular_speed)
      : modelName_(modelName),
        time(time),
        maxSpeed_(maxSpeed_),
        neighborDistance_(neighborDistance_),
        timeHorizon_(timeHorizon_),
        radius_(radius_),
        goal_pose(goal_pose),
        // target_model_state(target_model_state),
        lastStoredNewVelocity(agentVelocity),
        num(num),
        max_linear_speed(max_linear_speed),
        max_angular_speed(max_angular_speed),
        newVelocities(1, Vector2(0, 0))

  {
    // 初始化ROS节点
    ros::NodeHandle nh;
    target_model_ = modelName;
    model_states_sub_ = nh.subscribe("/gazebo/model_states", 10, &ModelSubPub::modelStatesCallback, this);
    model_states_pub_ = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10);
    pose_stamped_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/pose_stamped_topic", 10); // 新增的发布器
    path_pub_ = nh.advertise<nav_msgs::Path>("/path_topic", 10);                             // 新增的发布器
  }
  // 回调函数，处理模型状态信息
  void ModelSubPub::modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
  {
    std::cout << "8912342222132222222222222Moved to new_twist.linear.x=" << target_model_state.twist.linear.x << std::endl;
    other_models_states.clear();
    // gazebo_msgs::ModelState target_model_state;
    // 遍历所有模型
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
      if (msg->name[i] == target_model_)
      {
        target_model_state.model_name = msg->name[i];
        target_model_state.pose = msg->pose[i];
        target_model_state.twist = msg->twist[i];
      }
      else if (msg->name[i] != "ground_plane")
      {
        // 存储其他模型的信息
        gazebo_msgs::ModelState other_model_state;
        other_model_state.model_name = msg->name[i];
        other_model_state.pose = msg->pose[i];
        other_model_state.twist = msg->twist[i];
        other_models_states.push_back(other_model_state);
      }
    }
    std::string agentname = target_model_;
    agentpose = target_model_state.pose;
    agenttwist = target_model_state.twist;
    std::cout << "912342222132222222222222Moved to new_twist.linear.x=" << target_model_state.twist.linear.x << std::endl;
    // 格式转化
    Vector2 agentPosition(agentpose.position.x, agentpose.position.y);
    double deltaTheta = agenttwist.angular.z * time;
    // 这个是这一次的转换角度
    double velocityX = agenttwist.linear.x * cos(deltaTheta);
    double velocityY = agenttwist.linear.x * sin(deltaTheta);
    Vector2 agentVelocity(velocityX, velocityY);
    Vector2 goalPosition(goal_pose.position.x, goal_pose.position.y);

    obstacle_poses.clear(); // 清空 obstacle_poses，确保它是空的
    // 将 other_models_states 中的数据转移到 obstacle_poses 中
    for (const auto &model_state : other_models_states)
    {
      geometry_msgs::Pose pose = model_state.pose;
      obstacle_poses.push_back(pose);
    }

    RVO::DWAPlanner planner(goal_pose, obstacle_poses, max_linear_speed, max_angular_speed, time, num, agentpose);
    // 查找最佳速度，final_pose，最佳分数
    const geometry_msgs::Twist &best_twist = planner.FindBestTwist(agentpose);
    // 在这里得到返回值最佳速度，然后将这个转换为偏好速度，将输入信息改变
    double goalTheta = best_twist.angular.z * time;
    // 这个是这一次的转换角度
    double velocityX1 = best_twist.linear.x * cos(goalTheta);
    double velocityY1 = best_twist.linear.x * sin(goalTheta);
    Vector2 prefVelocity(velocityX1, velocityY1);
    RVO::Neighbor neighborobject(*this);
    // // 获取计算后的邻居信息
    std::vector<RVO::Agent *> agentNeighbors_ = neighborobject.getAgentNeighbors();
    std::vector<RVO::Obstacle *> obstacleNeighbors_ = neighborobject.getObstacleNeighbors();
    RVO::Agent agent(agentPosition, agentVelocity, prefVelocity, time, maxSpeed_, neighborDistance_, timeHorizon_, other_models_states, radius_);
    Vector2 newVelocity = agent.computeNewVelocity(agentPosition, agentVelocity, prefVelocity, agentNeighbors_, obstacleNeighbors_, time);
    geometry_msgs::Pose final_pose;
    if (std::isnan(newVelocity.x()) || std::isnan(newVelocity.y()))
    {
      final_pose.position.x = agentPosition.x();
      final_pose.position.y = agentPosition.y();
      std::cout << "New velocity contains NaN. Keeping original position." << std::endl;
    }
    else
    {
      if (newVelocity != lastStoredNewVelocity)
      {
        newVelocities.push_back(newVelocity); // 将上一次存储的速度放入容器
        lastStoredNewVelocity = newVelocity;  // 初始信息设置为0，也就是初始的位置不论朝向怎么样，都根据相关的速度计算得到角速度。
        // 速度改变，将旧的值给last
        lastvelocity = newVelocities[newVelocities.size() - 2];
        // 更新存储的新速度为当前计算得到的新速度
      }
      double initialtheta2 = atan2(lastvelocity.y(), lastvelocity.x());
      // std::cout << "32222222222222Moved to  initialtheta2: x=" << initialtheta2 << std::endl;
      // 先将相关的速度格式转换    std::cout << "433333Moved to new newVelocity x=" << newVelocity.x() << ", y=" << newVelocity.y() << std::endl;
      double X = newVelocity.x();
      double Y = newVelocity.y();
      new_twist.linear.x = sqrt(X * X + Y * Y);
      double theta = atan2(Y, X);
      new_twist.angular.x = 0;
      new_twist.angular.y = 0;
      new_twist.angular.z = (theta - 0) / time;
      // 引入运动学模型，现在是有了速度角速度，pose是传进来的初始信息，但速度角速度应该是新的计算结果
      KinematicModel kinematic_model(agentpose, new_twist);
      final_pose = kinematic_model.calculateNewPosition(time);
      // 得到新的速度
      //  final_pose.position.x = agentPosition.x() + newVelocity.x() * time;
      //  final_pose.position.y = agentPosition.y() + newVelocity.y() * time;
      std::cout << "Moved to new position: x=" << final_pose.position.x << ", y=" << final_pose.position.y << std::endl;
    }

    gazebo_msgs::ModelState model_state;
    model_state.model_name = agentname;
    model_state.pose = final_pose;
    model_states_pub_.publish(model_state);

    new_poses.push_back(new_pose);
    std::size_t size = new_poses.size();
    std::cout << "a111size:" << size << std::endl;

    // 发布pose信息
    geometry_msgs::PoseStamped pose_stamped_msg;
    pose_stamped_msg.header.stamp = ros::Time::now(); // 使用当前时间作为时间戳
    pose_stamped_msg.header.frame_id = "map";
    pose_stamped_msg.pose.position.x = new_pose.position.x;
    pose_stamped_msg.pose.position.y = new_pose.position.y;
    pose_stamped_msg.pose.orientation = new_pose.orientation;
    // 发布 geometry_msgs::PoseStamped 类型的消息
    pose_stamped_pub_.publish(pose_stamped_msg);

    // 发布path信息
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "map"; // 设置路径消息的坐标系
    // 需要设置较多的信息
    for (int i = 0; i < new_poses.size(); ++i)
    {
      // 添加路径点到路径消息中
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = ros::Time::now();
      pose.header.frame_id = "map"; // 设置路径点的坐标系
      pose.pose = new_poses[i];

      // 信息传输成功，但是值不是有效值；那么应该怎么设置
      // std::cout << "11111Moved to new position: x=" << new_poses[i].position.x << ", y=" << new_poses[i].position.y << std::endl;
      // pose.pose.id=modelName_;

      path_msg.poses.push_back(pose); // 将路径点添加到路径消息中
    }
    path_pub_.publish(path_msg); // 发布路径消息
  }

std::vector<gazebo_msgs::ModelState> ModelSubPub::getothermodels() const
{
  return other_models_states;
};
}
