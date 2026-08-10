#include <cmath>
#include "vicktor.hpp"
/*
Vicktor Vicktor::sum(Vicktor p, Vicktor q)
{
    Vicktor vector_update{};
    vector_update.x = p.x + q.x;
    vector_update.y = p.y + q.y;

    return vector_update;
}

Vicktor Vicktor::subtract(Vicktor p, Vicktor q)
{
    Vicktor vector_update{};
    vector_update.x = p.x - q.x;
    vector_update.y = p.y - q.y;
    return vector_update;
}

Vicktor Vicktor::scalar_multi(Vicktor p, double n)
{
    Vicktor vector_update{};
    vector_update.x = p.x * n;
    vector_update.y = p.y * n;
    return vector_update;
}

double Vicktor::module(Vicktor p)
{
    return sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
}
*/

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
    return std::sqrt(x * x + y * y);
}

// comodo anche fuori dalla classe, per scrivere n * v
inline Vicktor operator*(double n, const Vicktor &v) { return v * n; }