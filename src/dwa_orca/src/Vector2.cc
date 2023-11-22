

/**
 * @file  Vector2.cc
 * @brief Defines the Vector2 class.
 */
#include "Vector2.h"

#include <cmath>
#include <ostream>

namespace RVO {
const float RVO_EPSILON = 0.00001F;

Vector2::Vector2() : x_(0.0F), y_(0.0F) {}

Vector2::Vector2(float x, float y) : x_(x), y_(y) {}

Vector2 Vector2::operator-() const { return Vector2(-x_, -y_); }

float Vector2::operator*(const Vector2 &vector) const {
  return x_ * vector.x_ + y_ * vector.y_;
}

Vector2 Vector2::operator*(float scalar) const {
  return Vector2(x_ * scalar, y_ * scalar);
}

Vector2 Vector2::operator/(float scalar) const {
  const float invScalar = 1.0F / scalar;

  return Vector2(x_ * invScalar, y_ * invScalar);
}

Vector2 Vector2::operator+(const Vector2 &vector) const {
  return Vector2(x_ + vector.x_, y_ + vector.y_);
}

Vector2 Vector2::operator-(const Vector2 &vector) const {
  return Vector2(x_ - vector.x_, y_ - vector.y_);
}

bool Vector2::operator==(const Vector2 &vector) const {
  return x_ == vector.x_ && y_ == vector.y_;
}

bool Vector2::operator!=(const Vector2 &vector) const {
  return x_ != vector.x_ || y_ != vector.y_;
}
Vector2 &Vector2::operator*=(float scalar) {
  x_ *= scalar;
  y_ *= scalar;

  return *this;
}
Vector2 &Vector2::operator/=(float scalar) {
  const float invScalar = 1.0F / scalar;
  x_ *= invScalar;
  y_ *= invScalar;

  return *this;
}
Vector2 &Vector2::operator+=(const Vector2 &vector) {
  x_ += vector.x_;
  y_ += vector.y_;

  return *this;
}
Vector2 &Vector2::operator-=(const Vector2 &vector) {
  x_ -= vector.x_;
  y_ -= vector.y_;

  return *this;
}
Vector2 operator*(float scalar, const Vector2 &vector) {
  return Vector2(scalar * vector.x(), scalar * vector.y());
}
std::ostream &operator<<(std::ostream &stream, const Vector2 &vector) {
  stream << "(" << vector.x() << "," << vector.y() << ")";

  return stream;
}
float abs(const Vector2 &vector) { return std::sqrt(vector * vector); }

float absSq(const Vector2 &vector) { return vector * vector; }

float det(const Vector2 &vector1, const Vector2 &vector2) {
  return vector1.x() * vector2.y() - vector1.y() * vector2.x();
}

float leftOf(const Vector2 &vector1, const Vector2 &vector2,
             const Vector2 &vector3) {
  return det(vector1 - vector3, vector2 - vector1);
}
Vector2 normalize(const Vector2 &vector) { return vector / abs(vector); }
} /* namespace RVO */
