#ifndef N_BODIES_HPP
#define N_BODIES_HPP
#include <cmath>
#include <vector>
#include <iostream>
#include <string>
#include <limits>
#include <algorithm>
#include <random>
#include "vector2d.hpp"
namespace nb
{
    class Planet
    {
        double mass;
        double radius;

    public:
        Vector2d position;
        Vector2d velocity;
        Vector2d acceleration;
        Planet(double m, double posx, double posy, double velx, double vely, double r, double accex = 0.0, double accey = 0.0)
            : mass(m), radius(r)
        {
            setMass(m);
            setRadius(r);
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
            if (newMass < 0.0)
            {
                throw std::invalid_argument("La massa non può essere negativa.");
            }
            mass = newMass;
        }
        void setRadius(double newRadius)
        {
            if (newRadius < 0.0)
            {
                throw std::invalid_argument("Il raggio non deve essere negativo.");
            }
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
        static constexpr double pi = 3.14159265358979323846;
        std::default_random_engine eng;
        std::uniform_real_distribution<int> uniform{2, 6};
        std::vector<Planet> bodies;

    public:
        MinMaxTracker energyRange;
        MinMaxTracker angularMomentumRange;
        MinMaxTracker momentumRange;
        void loadFromFile(const std::string &filename);
        void addBody(double m, double posx, double posy, double velx, double vely, double r);
        size_t numBodies() const { return bodies.size(); }
        const Planet &getBody(size_t i) const { return bodies[i]; }
        std::vector<Planet>::const_iterator begin() const { return bodies.cbegin(); }
        std::vector<Planet>::const_iterator end() const { return bodies.cend(); }

        double consEnergy() const;
        Vector2d centreOfMass() const;
        double totalMass() const;
        double consAngularMomentum() const;
        Vector2d consMomentum() const;
        void initAccelerations();
        bool explosiveCollision(long unsigned int i, long unsigned int j) const;
        void step(double dt);
        Vector2d lagrange(int i) const;
    };
}

#endif
