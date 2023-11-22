#ifndef RVO_LINE_H_
#define RVO_LINE_H_

/**
 * @file  Line.h
 * @brief Declares the Line class.
 */

#include "Vector2.h"

namespace RVO {
/**
 * @brief Defines a directed line.
 */
class RVO_EXPORT Line {
 public:
  /**
   * @brief Constructs a directed line instance.
   */
  Line();

  /**
   * @brief The direction of the directed line.
   */
  Vector2 direction;

  /**
   * @brief A point on the directed line.
   */
  Vector2 point;
};
} /* namespace RVO */

#endif /* RVO_LINE_H_ */
