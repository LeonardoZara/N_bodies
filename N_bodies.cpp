
#include <cmath>
#include <vector>
#include <iostream>
#include "vicktor.hpp"
#include "N_bodies.hpp"
#include "body.hpp"

void Simulation::step(double dt)
{
    for (size_t i = 0; i < bodies.size(); ++i)
    {
        bodies[i].position = bodies[i].position.sum(bodies[i].position.sum(bodies[i].position, bodies[i].velocity.scalar_multi(bodies[i].velocity, dt)), bodies[i].acceleration.scalar_multi(bodies[i].acceleration, 0.5 * dt * dt));
        Vicktor acc = bodies[i].acceleration;
        bodies[i].acceleration.x = 0;
        bodies[i].acceleration.y = 0;
        for (size_t j = 0; j < bodies.size(); ++j)
        {
            if (j != i)
            {
                bodies[i].acceleration.x = G * bodies[i].getMass() * (bodies[i].position.x - bodies[j].position.x) / pow(pow(bodies[j].position.x - bodies[i].position.x, 2) + pow(epsilon, 2), 1.5);
                bodies[i].acceleration.y = G * bodies[i].getMass() * (bodies[i].position.y - bodies[j].position.y) / pow(pow(bodies[j].position.y - bodies[i].position.y, 2) + pow(epsilon, 2), 1.5);
            }
        }
        bodies[i].acceleration = bodies[i].acceleration.scalar_multi(bodies[i].acceleration, -1);
        bodies[i].velocity = bodies[i].velocity.sum(bodies[i].velocity, bodies[i].velocity.scalar_multi(bodies[i].velocity.sum(acc, bodies[i].acceleration), 0.5 * dt));
    }
}