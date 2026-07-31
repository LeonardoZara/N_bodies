#ifndef BODY_HPP
#define BODY_HPP
#include <cmath>
#include <vector>
#include "vicktor.hpp"


class Planet
{
    double mass;
    double diameter{};
    double const epsilon{pow(10, -12)};

    public:
    Vicktor position;
    Vicktor velocity;
    Vicktor acceleration;
    Planet (double posx, double velx, double accex, double posy, double vely, double accey){
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

    double getEpsilon() const {
        return epsilon;
    }

};
#endif