#ifndef N_BODIES_HPP
#define N_BODIES_HPP
#include <cmath>
#include <vector>
#include <iostream>
#include <string>
#include "vicktor.hpp"

class Planet
{
    double mass;
    double radius;
public:
    Vicktor position;
    Vicktor velocity;
    Vicktor acceleration;
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
    void setMass(double newMass)
    {
        mass = newMass;
    }
    void setRadius(double newRadius)
    {
        radius = newRadius;
    }
};

struct MinMaxTracker
{
    double min{std::numeric_limits<double>::infinity()};
    double max{-std::numeric_limits<double>::infinity()};

    void update(double v)
    {
        min = std::min(min, v);
        max = std::max(max, v);
    }
};

class Simulation
{
    static constexpr double G{6.6743e-11};

public:
    std::vector<Planet> bodies;
    MinMaxTracker energyRange;
    MinMaxTracker angularMomentumRange;
    MinMaxTracker momentumRange;
    void loadFromFile(const std::string &filename);
    double consEnergy() const;
    Vicktor centreOfMass() const;
    double totalMass() const;
    double consAngularMomentum() const;
    Vicktor consMomentum() const;
    void initAccelerations();
    void step(double dt);
    Vicktor lagrange(int i) const;
};

#endif
