#ifndef BODY_HPP
#define BODY_HPP
#include <cmath>
#include <vector>
#include "vicktor.hpp"


class Planet
{
    double diameter{};

    public:
    Vicktor position;
    Vicktor velocity;
    Vicktor acceleration;
    double mass;
     Planet (double m, double posx, double posy, double velx, double vely, double accex = 0.0, double accey = 0.0)
    : mass(m)
    {
        position.x=posx;
        position.y=posy;
        velocity.x=velx;
        velocity.y=vely;
        acceleration.x=accex;
        acceleration.y=accey;
    }

    double getMass() const {
        return mass;
    }
};
#endif