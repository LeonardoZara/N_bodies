#include "vicktor.hpp"
#include <cmath>
#include <vector>

class Planet
{
    double mass;
    double diameter{};
    double const epsilon{pow(10, -12)};

    public:
    Vicktor position;
    Vicktor velocity;
    Vicktor acceleration;
    Planet (){
        position.x=0;
        position.y=0;
        velocity.x=0;
        velocity.y=0;
        acceleration.x=0;
        acceleration.y=0;
    }

    double getMass() const {
        return mass;
    }

    double getEpsilon() const {
        return epsilon;
    }

};

