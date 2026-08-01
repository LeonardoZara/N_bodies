#ifndef N_BODIES_HPP
#define N_BODIES_HPP
#include <cmath>
#include <vector>
#include <iostream>
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

class Simulation
{
    double const G{6.6743e-11};
    double const epsilon{1e-12};
    public:
    std::vector<Planet> bodies;
    std::vector<double> energiesHistory;
    std::vector<double> angularMomentumHistory;
    double consEnergy();
    double consAngularMomentum();
    void initAccelerations();
    void step(double dt);
};
#endif