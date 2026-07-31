#ifndef N_BODIES_HPP
#define N_BODIES_HPP
#include <cmath>
#include <vector>
#include <iostream>
#include "vicktor.hpp"
#include "body.hpp"


class Simulation
{
    double const G{66743 * 10 ^ (-15)};
    double const epsilon{pow(10, -12)};
    public:
    std::vector<Planet> bodies;

    void step(double dt);
};
#endif