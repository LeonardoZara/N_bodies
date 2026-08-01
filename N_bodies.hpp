#ifndef N_BODIES_HPP
#define N_BODIES_HPP
#include <cmath>
#include <vector>
#include <iostream>
#include "vicktor.hpp"
#include "body.hpp"


class Simulation
{
    double const G{6.6743e-11};
    double const epsilon{1e-12};
    public:
    std::vector<Planet> bodies;
 
    void initAccelerations();
    void step(double dt);
};
#endif