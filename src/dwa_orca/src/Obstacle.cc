
#include "Obstacle.h"

namespace RVO {
Obstacle::Obstacle()
    : next_(NULL), previous_(NULL), id_(0U), isConvex_(false) {}

Obstacle::~Obstacle() {}
} /* namespace RVO */
