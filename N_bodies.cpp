#include "N_bodies.hpp"
#include <cmath>
#include <vector>
#include <iostream>
class Planet
{
    double mass{};
    double pos_x{};
    double pos_y{};
    double vel_x{};
    double vel_y{};
    double acc_x{};
    double acc_y{};

    double const epsilon{pow(10, -12)};
};

std::vector<Planet> Bodies;