#ifndef BODY_HPP
#define BODY_HPP
#include <cmath>
#include <vector>
#include "vicktor.hpp"


class Planet
{
    double diameter{};
    double const epsilon{pow(10, -12)};

    public:
    Vicktor position;
    Vicktor velocity;
    Vicktor acceleration;
    double mass;
    Planet (double m, double posx, double velx, double accex = 0., double posy, double vely, double accey=0.){
        mass = m;
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