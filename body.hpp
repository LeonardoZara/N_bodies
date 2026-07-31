#include "vicktor.hpp"
#include <cmath>
#include <vector>

class Planet
{
    double mass{};
    double pos_x{};
    double pos_y{};
    double vel_x{};
    double vel_y{};
    double acc_x{};
    double acc_y{};
    double diameter{};

    Vicktor pos;
    pos.x = pos_x;
    double const epsilon{pow(10, -12)};
};

