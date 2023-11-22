
#include <algorithm>
#include <utility>
#include "Agent.h"
#include "Obstacle.h"
#include "Vector2.h"
#include "Neighbor.h"

namespace RVO {
    Neighbor::Neighbor(const ModelSubPub & modelSubPub) : modelSubPub_(modelSubPub) {
        std::vector<gazebo_msgs::ModelState> other_models_states = modelSubPub_.getothermodels();
        if (!other_models_states.empty()) {
            ROS_INFO("Number of other models: %zu", other_models_states.size());
            for (const gazebo_msgs::ModelState& model_state : other_models_states) {
                if (isAgent(model_state)) {
                    const geometry_msgs::Pose &pose = model_state.pose;
                    const geometry_msgs::Twist &twist = model_state.twist;
                    Vector2 point(pose.position.x, pose.position.y);
                    double deltaTheta = twist.angular.z * time;
                    double velocityX = twist.linear.x * cos(deltaTheta);
                    double velocityY = twist.linear.x * sin(deltaTheta);
                    Vector2 velocity_(velocityX, velocityY);
                    Agent* newAgent = new Agent(point, velocity_);
                    agentNeighbors_.push_back(newAgent);
                    //智能体是可以的
                } else if (isObstacle(model_state)) {
                    const geometry_msgs::Pose &pose = model_state.pose;
                    const geometry_msgs::Twist &twist = model_state.twist;
                    Vector2 position(pose.position.x, pose.position.y);
                    double deltaTheta = twist.angular.z * time;
                    double velocityX = twist.linear.x * cos(deltaTheta);
                    double velocityY = twist.linear.x * sin(deltaTheta);
                    Vector2 velocity_(velocityX, velocityY);
                    Obstacle* newObstacle = new Obstacle(position, velocity_);
                    obstacleNeighbors_.push_back(newObstacle);
                }
            }
        } else {
            ROS_INFO("No other models received.");
        }
    }
    std::vector<Agent*> Neighbor::getAgentNeighbors() const {
        return agentNeighbors_;
    }
    std::vector<Obstacle*> Neighbor::getObstacleNeighbors() const {
        return obstacleNeighbors_;
    }
    bool Neighbor::isAgent(const gazebo_msgs::ModelState& model_state) {
        return true;
    }
    bool Neighbor::isObstacle(const gazebo_msgs::ModelState& model_state) {
        return true;
    }
}
