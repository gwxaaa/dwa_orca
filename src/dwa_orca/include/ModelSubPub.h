#ifndef MODEL_SUB_PUB_H
#define MODEL_SUB_PUB_H
#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/SetModelState.h"
#include "std_msgs/String.h"
#include "Agent.h"
#include "Obstacle.h"
#include "Neighbor.h"
namespace RVO
{

  class ModelSubPub
  {
  public:
    ModelSubPub(const std::string &modelName, double time, gazebo_msgs::ModelState target_model_state, geometry_msgs::Pose goal_pose,
                double maxSpeed_, double neighborDistance_, double timeHorizon_, double radius);
    // 回调函数，处理模型状态信息
    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg);
    std::vector<gazebo_msgs::ModelState> getothermodels() const;
  private:
    ros::NodeHandle nh;
    ros::Subscriber model_states_sub_;
    ros::Publisher model_states_pub_;
    std::string target_model_; // 用于存储目标模型的名称
    double time;
    geometry_msgs::Pose target_model_pose;
    double new_velocity;
    geometry_msgs::Pose agentpose;
    geometry_msgs::Twist agenttwist;
    geometry_msgs::Pose goal_pose;
    std::vector<gazebo_msgs::ModelState> other_models_states;
    std::string modelName_;
    double time_;
    double targetModelSpeed_;
    double goalModelSpeed_;
    double maxSpeed_;
    double neighborDistance_;
    double timeHorizon_;
    double radius_; // 避障半径
    std::vector<Agent *> agentNeighbors_;
    std::vector<Agent *> obstacleNeighbors_;
    gazebo_msgs::ModelState target_model_state;
    Vector2 agentPosition;
    Vector2 agentVelocity;
    Vector2 goalPosition;
    geometry_msgs::Pose new_pose;

  };
} // namespace RVO
#endif // MODEL_SUB_PUB_H