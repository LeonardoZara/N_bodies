#include <cmath>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include "vicktor.hpp"
#include "N_bodies.hpp"

void Simulation::loadFromFile(const std::string &filename)
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        throw std::runtime_error("Impossibile aprire il file: " + filename);
    }
    bodies.clear();
    std::string line;
    while (std::getline(file, line))
    {
        if (line.empty() || line[0] == '#')
        {
            continue;
        }

        std::istringstream iss(line);
        double m, posx, posy, velx, vely;
        if (!(iss >> m >> posx >> posy >> velx >> vely))
            throw std::runtime_error("Riga malformata nel file bodies: " + line);

        bodies.emplace_back(m, posx, posy, velx, vely);
    }

    if (bodies.empty())
        throw std::runtime_error("Nessun corpo caricato dal file: " + filename);
}

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

double Simulation::consEnergy()
{
    // Conservazione energia:
    double k{0};
    double u{0};
    for (size_t i = 0; i < bodies.size(); ++i)
    {
        k += 0.5 * bodies[i].getMass() * pow(bodies[i].velocity.module(bodies[i].velocity), 2);
    }
    for (size_t j = 1; j < bodies.size(); ++j)
    {
        for (size_t i = 0; i < j; ++i)
        {
            Vicktor distance = bodies[i].position.subtract(bodies[i].position, bodies[j].position);
            u += -G * bodies[i].getMass() * bodies[j].getMass() / bodies[i].position.module(distance);
        }
    }
    return k + u;
}

Vicktor Simulation::centreOfMass()
{
    Vicktor cm{0, 0};
    Vicktor cmNumerator{0., 0.};
    double totalMass{0.};
    for (size_t i = 0; i < bodies.size(); ++i)
    {
        cmNumerator = cm.sum(cm, cm.scalar_multi(bodies[i].position, bodies[i].getMass()));
        totalMass += bodies[i].getMass();
    }
    cm = cm.scalar_multi(cmNumerator, (1 / totalMass));
    return cm;
}

double Simulation::totalMass()
{
    double totalMass{0.};
    for (size_t i = 0; i < bodies.size(); ++i)
    {
        totalMass += bodies[i].getMass();
    }
    return totalMass;
}

double Simulation::consAngularMomentum()
{
    Vicktor cm = centreOfMass();
    double angularMomentum{0};
    for (size_t i = 0; i < bodies.size(); ++i)
    {
        angularMomentum += bodies[i].getMass() * (((bodies[i].position.x - cm.x) * bodies[i].velocity.y) - (((bodies[i].position.y - cm.y) * bodies[i].velocity.x)));
    }
    return angularMomentum;
}

Vicktor Simulation::consMomentum()
{
    Vicktor momentum{0., 0.};

    for (size_t i = 0; i < bodies.size(); ++i)
    {
        momentum = momentum.sum(momentum, bodies[i].velocity.scalar_multi(bodies[i].velocity, bodies[i].getMass()));
    }
    return momentum;
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
    energiesHistory.push_back(consEnergy());
    angularMomentumHistory.push_back(consAngularMomentum());
    momentumHistory.push_back(consMomentum().module(consMomentum()));
}