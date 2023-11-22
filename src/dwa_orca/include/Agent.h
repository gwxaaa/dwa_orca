#ifndef RVO_AGENT_H_
#define RVO_AGENT_H_
/**
 * @file  Agent.h
 * @brief Declares the Agent class.
 */

#include <cstddef>
#include <utility>
#include <vector>
#include "Line.h"
#include "Vector2.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/ModelState.h"
#include "ModelSubPub.h"
#include "Neighbor.h"
namespace RVO
{
  class Neighbor;
  class Obstacle;
  class Vector2;
  class Agent
  {

  public:
    /**
     * @brief Constructs an agent instance.
     */
    Agent() {}
    Agent(const Vector2 &position, const Vector2 &velocity) : position_(position), velocity_(velocity) {}
    Agent(const Vector2& agentPosition, const Vector2& agentVelocity,const Vector2& goalPosition,double time,
          double maxSpeed_, double neighborDistance_, double timeHorizon_, const  std::vector<gazebo_msgs::ModelState> other_models_states,
          double radius_);
    ~Agent();
    void computeNeighbors(const Neighbor *neighbor);
    Vector2 computeNewVelocity(const Vector2& agentPosition, const Vector2& agentVelocity,
                                  const Vector2& goalPosition,
                                  const std::vector<RVO::Agent*>& agentNeighbors_,
                                  const std::vector<RVO::Obstacle*>& obstacleNeighbors_,
                                  double time);

    void update(float timeStep);
    std::vector<RVO::Agent *> agentNeighbors_;
    std::vector<RVO::Obstacle *> obstacleNeighbors_;
    std::vector<Line> orcaLines_;
    Vector2 newVelocity_;
    Vector2 position_;
    Vector2 prefVelocity_;
    Vector2 velocity_;
    std::size_t id_;
    std::size_t maxNeighbors_;
    geometry_msgs::Pose agentpose;
    geometry_msgs::Pose goal_pose;
    geometry_msgs::Twist agenttwist;
    double maxSpeed_;
    double neighborDist_;
    double radius_;
    double timeHorizon_;
    double timeHorizonObst_;
    double time;
  };
} /* namespace RVO */

#endif /* RVO_AGENT_H_ */
