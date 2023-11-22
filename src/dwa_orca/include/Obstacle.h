#ifndef RVO_OBSTACLE_H_
#define RVO_OBSTACLE_H_

/**
 * @file  Obstacle.h
 * @brief Declares the Obstacle class.
 */

#include <cstddef>

#include "Vector2.h"

namespace RVO {
/**
 * @brief Defines static obstacles in the simulation.
 */
class Obstacle {
 public:
  /**
   * @brief Constructs a static obstacle instance.
   */
  Obstacle();
   Obstacle(const Vector2& point, const Vector2& velocity){}
  /**
   * @brief Destroys this static obstacle instance.
   */
  ~Obstacle();

  /* Not implemented. */
  Obstacle(const Obstacle &other);

  /* Not implemented. */
  Obstacle &operator=(const Obstacle &other);

  Vector2 direction_;
  Vector2 point_;
  Obstacle *next_;
  Obstacle *previous_;
  std::size_t id_;
  bool isConvex_;
  friend class Agent;
  friend class KdTree;
};
} /* namespace RVO */

#endif /* RVO_OBSTACLE_H_ */
