#include <iostream>
#include <cmath>

struct Vicktor
{
    double x{};
    double y{};
};

struct Operations
{
    Vicktor sum(Vicktor p, Vicktor q)
    {
        Vicktor vector_update{};
        vector_update.x = p.x + q.x;
        vector_update.y = p.y + q.y;

        return vector_update;
    }

    Vicktor subtract(Vicktor p, Vicktor q)
    {
        Vicktor vector_update{};
        vector_update.x = p.x - q.x;
        vector_update.y = p.y - q.y;
        return vector_update;
    }

    Vicktor scalar_multi(Vicktor p, double n)
    {
        Vicktor vector_update{};
        vector_update.x = p.x * n;
        vector_update.y = p.y * n;
        return vector_update;
    }

    double module(Vicktor p)
    {
        return sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
    }
};
