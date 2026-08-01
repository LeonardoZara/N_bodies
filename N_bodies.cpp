#include <cmath>
#include <vector>
#include <iostream>
#include "vicktor.hpp"
#include "N_bodies.hpp"

static Vicktor gravAcceleration(const std::vector<Planet> &bodies, size_t i, double G, double epsilon)
{
    Vicktor acc{};
    for (size_t j = 0; j < bodies.size(); ++j)
    {
        if (j == i)
            continue;

        double dx = bodies[j].position.x - bodies[i].position.x;
        double dy = bodies[j].position.y - bodies[i].position.y;
        double denom = pow(dx * dx + dy * dy + epsilon * epsilon, 1.5);

        acc.x += G * bodies[j].getMass() * dx / denom;
        acc.y += G * bodies[j].getMass() * dy / denom;
    }
    return acc;
}

void Simulation::initAccelerations()
{
    for (size_t i = 0; i < bodies.size(); ++i)
    {
        bodies[i].acceleration = gravAcceleration(bodies, i, G, epsilon);
    }
}

void Simulation::step(double dt)
{
    for (size_t i = 0; i < bodies.size(); ++i)
    {
        bodies[i].position = bodies[i].position.sum(
            bodies[i].position.sum(bodies[i].position, bodies[i].velocity.scalar_multi(bodies[i].velocity, dt)),
            bodies[i].acceleration.scalar_multi(bodies[i].acceleration, 0.5 * dt * dt));
    }

    for (size_t i = 0; i < bodies.size(); ++i)
    {
        Vicktor acc_old = bodies[i].acceleration;
        Vicktor acc_new = gravAcceleration(bodies, i, G, epsilon);
        Vicktor acc_sum = acc_old.sum(acc_old, acc_new);

        bodies[i].velocity = bodies[i].velocity.sum(bodies[i].velocity, bodies[i].velocity.scalar_multi(acc_sum, 0.5 * dt));
        bodies[i].acceleration = acc_new;
    }
}