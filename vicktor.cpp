#include <cmath>
#include "vicktor.hpp"

namespace nb{
Vicktor Vicktor::operator+(const Vicktor &q) const
{
    return {x + q.x, y + q.y};
}
Vicktor Vicktor::operator-(const Vicktor &q) const
{
    return {x - q.x, y - q.y};
}
Vicktor Vicktor::operator*(double n) const
{
    return {x * n, y * n};
}
Vicktor &Vicktor::operator+=(const Vicktor &q)
{
    x += q.x;
    y += q.y;
    return *this;
}

double Vicktor::module() const
{
    return sqrt(x * x + y * y);
}
}