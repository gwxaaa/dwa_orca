#ifndef MODEL_SUB_PUB_H
#define MODEL_SUB_PUB_H
#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/SetModelState.h"
#include "std_msgs/String.h"
#include "Agent.h"
#include "Obstacle.h"
#include "Neighbor.h"
#include <KinematicModel.h>
#include <geometry_msgs/PoseStamped.h>
#include "nav_msgs/Path.h"
namespace RVO
{

  class ModelSubPub
  {
  public:
    ModelSubPub(const std::string &modelName, double time, gazebo_msgs::ModelState target_model_state, geometry_msgs::Pose goal_pose,
                double maxSpeed_, double neighborDistance_, double timeHorizon_, double radius_, double num, double max_linear_speed, double max_angular_speed);
    // 回调函数，处理模型状态信息
    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg);
    std::vector<gazebo_msgs::ModelState> getothermodels() const;
    double radius_; // 避障半径
  private:
    ros::NodeHandle nh;
    ros::Subscriber model_states_sub_;
    ros::Publisher model_states_pub_;
    std::string target_model_; // 用于存储目标模型的名称
    double time;
    double num;
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
    std::vector<Agent *> agentNeighbors_;
    std::vector<Agent *> obstacleNeighbors_;
    gazebo_msgs::ModelState target_model_state;
    Vector2 agentPosition;
    Vector2 agentVelocity;
    Vector2 goalPosition;
    geometry_msgs::Pose new_pose;
    geometry_msgs::Twist new_twist;
    geometry_msgs::Pose final_pose;
    Vector2 lastvelocity;
    Vector2 prevelocity;
    Vector2 lastStoredNewVelocity;
    std::vector<Vector2> newVelocities;
    double max_linear_speed;
    double max_angular_speed; // 用于存储新速度的数组
    std::vector<geometry_msgs::Pose> obstacle_poses;
    ros::Publisher pose_stamped_pub_;
    ros::Publisher path_pub_;
    std::vector<geometry_msgs::Pose> new_poses;
  };
} // namespace RVO
#endif // MODEL_SUB_PUB_H