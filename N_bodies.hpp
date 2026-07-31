#ifndef N_BODIES_HPP
#define N_BODIES_HPP

#include "vicktor.hpp"
#include "body.hpp"
#include <cmath>
#include <vector>
#include <iostream>
double const G{66743 * 10 ^ (-15)};
class Simulation
{
    double const epsilon{pow(10, -12)};
    public:
    std::vector<Planet> bodies;

    void step(double dt){};
};

#endif