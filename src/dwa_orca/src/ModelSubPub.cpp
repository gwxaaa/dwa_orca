
#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/ModelState.h"
#include "std_msgs/String.h"
#include "ModelSubPub.h"
#include "Agent.h"
#include "Neighbor.h"
#include <cmath>
#include <KinematicModel.h>
namespace RVO
{
  ModelSubPub::ModelSubPub(const std::string &modelName, double time, gazebo_msgs::ModelState target_model_state,
                           geometry_msgs::Pose goal_pose, double maxSpeed_, double neighborDistance_, double timeHorizon_, double radius_)
      : modelName_(modelName),
        time(time),
        maxSpeed_(maxSpeed_),
        neighborDistance_(neighborDistance_),
        timeHorizon_(timeHorizon_),
        radius_(radius_),
        goal_pose(goal_pose),
        target_model_state(target_model_state),
        lastvelocity(agentVelocity)
  {
    // 初始化ROS节点
    ros::NodeHandle nh;
    target_model_ = modelName;
    model_states_sub_ = nh.subscribe("/gazebo/model_states", 10, &ModelSubPub::modelStatesCallback, this);
    model_states_pub_ = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10);
  }
  // 回调函数，处理模型状态信息
  void ModelSubPub::modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
  {
    other_models_states.clear();
    // gazebo_msgs::ModelState target_model_state;
    // 遍历所有模型
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
      if (msg->name[i] == target_model_)
      {
        // 存储特定目标模型的信
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
    // 格式转化
    Vector2 agentPosition(agentpose.position.x, agentpose.position.y);
    double deltaTheta = agenttwist.angular.z * time;
    // 这个是这一次的转换角度
    //  根据新的朝向角度和线速度计算速度向量
    double velocityX = agenttwist.linear.x * cos(deltaTheta);
    double velocityY = agenttwist.linear.x * sin(deltaTheta);
    Vector2 agentVelocity(velocityX, velocityY);
    Vector2 goalPosition(goal_pose.position.x, goal_pose.position.y);
    // 如果借助agent,那么就会订阅一次信息就修改一次，然后就会修改角速度大小，所以应该是借助第一次的计算值进行相关的运行，在实现新速度之前，角速度线速度就是这个值
    //  计算上一次的角度与坐标轴的大小
    double initialtheta = atan2(velocityY, velocityX);
    std::cout << "32222222222222Moved to new position: x=" << initialtheta << std::endl;
    double initialtheta1 = atan2(lastvelocity.y(), lastvelocity.x());
    std::cout << "1232222222222222Moved to new position: x=" << initialtheta << std::endl;

    RVO::Neighbor neighborobject(*this);
    // // 获取计算后的邻居信息
    std::vector<RVO::Agent *> agentNeighbors_ = neighborobject.getAgentNeighbors();
    std::vector<RVO::Obstacle *> obstacleNeighbors_ = neighborobject.getObstacleNeighbors();
    RVO::Agent agent(agentPosition, agentVelocity, goalPosition, time, maxSpeed_, neighborDistance_, timeHorizon_, other_models_states, radius_);
    Vector2 newVelocity = agent.computeNewVelocity(agentPosition, agentVelocity, goalPosition, agentNeighbors_, obstacleNeighbors_, time);
    geometry_msgs::Pose final_pose;

    if (agentVelocity != lastvelocity)
    {
      lastvelocity = lastvelocity;
    } // 假设 agentVelocity 是上一次计算得到的速度矢量
    else if (agentVelocity == newVelocity)//实现了期望的速度，那么就会计算新的速度，
    {
      lastvelocity = agentVelocity;
    }
    else if (agentVelocity == lastvelocity) // 初始值时
    {
      lastvelocity = agentVelocity;
    }

    if (std::isnan(newVelocity.x()) || std::isnan(newVelocity.y()))
    {
      final_pose.position.x = agentPosition.x();
      final_pose.position.y = agentPosition.y();
      std::cout << "New velocity contains NaN. Keeping original position." << std::endl;
    }
    else
    {
      // 先将相关的速度格式转换
      double X = newVelocity.x();
      double Y = newVelocity.y();
      new_twist.linear.x = sqrt(X * X + Y * Y);
      // 计算得到新的角度---新的速度下的角度
      double theta = atan2(Y, X);
      std::cout << "42222222222222Moved to new : x=" << theta << std::endl;

      // 一开始传进来的角度，和现在的角度作插值，
      // 前面的角度是在周期内，模型的转动角度，并不是那个时刻与初开始的，所以需要重新计算
      // new_twist.angular.z = theta/ time;
      new_twist.angular.z = (theta - initialtheta1) / time;
      new_twist.angular.x = 0;
      new_twist.angular.y = 0;
      std::cout << "2222222222222Moved to new : x=" << new_twist.linear.x << std::endl;
      std::cout << "11111111111Moved to new: x=" << new_twist.angular.z << std::endl;
      // 引入运动学模型，现在是有了速度角速度，pose是传进来的初始信息，但速度角速度应该是新的计算结果
      KinematicModel kinematic_model(agentpose, new_twist);
      final_pose = kinematic_model.calculateNewPosition(time);
      // 得到新的速度
      //  new_pose.position.x = agentPosition.x() + newVelocity.x() * time;
      //  new_pose.position.y = agentPosition.y() + newVelocity.y() * time;
      std::cout << "Moved to new position: x=" << final_pose.position.x << ", y=" << final_pose.position.y << std::endl;
    }

    gazebo_msgs::ModelState model_state;
    model_state.model_name = agentname;
    model_state.pose = final_pose;
    model_states_pub_.publish(model_state);
  }
  std::vector<gazebo_msgs::ModelState> ModelSubPub::getothermodels() const
  {
    return other_models_states;
  };
}
