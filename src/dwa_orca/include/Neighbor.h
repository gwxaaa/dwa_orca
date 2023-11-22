
#pragma once
#ifndef NEIGHBOR_H
#define NEIGHBOR_H
#include "ModelSubPub.h"
#include "Agent.h"
#include "Obstacle.h"
#include "Vector2.h"
#include <vector>
#include "gazebo_msgs/ModelState.h"

namespace RVO
{
    class ModelSubPub;
    class Agent;

    class Neighbor
    {
    public:
        Neighbor(const ModelSubPub &modelSubPub);
        // 获取计算后的邻居信息
        std::vector<Agent *> getAgentNeighbors() const;
        std::vector<Obstacle *> getObstacleNeighbors() const;
    private:
        const ModelSubPub &modelSubPub_;
        // 代理和障碍物的向量
        double time;
        std::vector<Agent *> agentNeighbors_;
        std::vector<Obstacle *> obstacleNeighbors_;
        std::vector<gazebo_msgs::ModelState> other_models_states;
        bool isAgent(const gazebo_msgs::ModelState &model_state);
        bool isObstacle(const gazebo_msgs::ModelState &model_state);
    };

} // namespace RVO

#endif // NEIGHBOR_H