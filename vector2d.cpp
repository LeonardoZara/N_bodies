#include <cmath>
#include "vector2d.hpp"

namespace nb{
Vector2d Vector2d::operator+(const Vector2d &q) const
{
    return {x + q.x, y + q.y};
}
Vector2d Vector2d::operator-(const Vector2d &q) const
{
    return {x - q.x, y - q.y};
}
Vector2d Vector2d::operator*(double n) const
{
    return {x * n, y * n};
}
Vector2d &Vector2d::operator+=(const Vector2d &q)
{
    x += q.x;
    y += q.y;
    return *this;
}

double Vector2d::module() const
{
    return sqrt(x * x + y * y);
}
}