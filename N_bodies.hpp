#ifndef N_BODIES_HPP
#define N_BODIES_HPP
#include <cmath>
#include <vector>
#include <iostream>
#include <string>
#include "vicktor.hpp"

class Planet
{

public:
    Vicktor position;
    Vicktor velocity;
    Vicktor acceleration;
    double mass;
    double radius;
    Planet(double m, double posx, double posy, double velx, double vely, double r, double accex = 0.0, double accey = 0.0)
        : mass(m), radius(r)
    {
        position.x = posx;
        position.y = posy;
        velocity.x = velx;
        velocity.y = vely;
        acceleration.x = accex;
        acceleration.y = accey;
    }

    double getMass() const
    {
        return mass;
    }
    double getRadius() const
    {
        return radius;
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
    std::vector<double> momentumHistory;
    void loadFromFile(const std::string &filename);
    double consEnergy();
    Vicktor centreOfMass();
    double totalMass();
    double consAngularMomentum();
    Vicktor consMomentum();
    void initAccelerations();
    void step(double dt);
    Vicktor lagrange(int i);
};

#endif
