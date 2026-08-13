#include <cmath>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <type_traits>
#include <numeric>
#include <algorithm>
#include <functional>
#include "vicktor.hpp"
#include "N_bodies.hpp"

namespace nb{
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
        double m, posx, posy, velx, vely, radius;
        if (!(iss >> m >> posx >> posy >> velx >> vely >> radius))
        {
            throw std::invalid_argument("Riga malformata nel file dati: " + line);
        }
        if (m < 0)
        {
            throw std::invalid_argument("La massa non può essere negativa." + line);
        }
        if (radius <= 0)
        {
            throw std::invalid_argument("Il raggio deve essere positivo." + line);
        }
        if (velx >= 3e8 || vely >= 3e8 || velx <= -3e8 || vely <= -3e8)
        {
            throw std::invalid_argument("La velocità dei corpi deve essere minore di quella della luce." + line);
        }
        bodies.emplace_back(m, posx, posy, velx, vely, radius);
    }

    if (bodies.empty())
        throw std::runtime_error("Nessun corpo caricato dal file: " + filename);
}

static Vicktor gravAcceleration(const std::vector<Planet> &bodies, size_t i, double G)
{
    Vicktor acc{};
    for (size_t j = 0; j < bodies.size(); ++j)
    {
        if (j == i)
            continue;

        double dx = bodies[j].position.x - bodies[i].position.x;
        double dy = bodies[j].position.y - bodies[i].position.y;
        double denom = pow(dx * dx + dy * dy, 1.5);

        acc.x += G * bodies[j].getMass() * dx / denom;
        acc.y += G * bodies[j].getMass() * dy / denom;
    }
    return acc;
}

double Simulation::consEnergy() const
{

    double u{0.};
    double k{0.};
    k = std::accumulate(bodies.begin(), bodies.end(), 0.0, [](double kin, const Planet &p)
                        { return kin + (0.5 * p.getMass() * pow(p.velocity.module(), 2)); });

    for (size_t j = 1; j < bodies.size(); ++j)
    {
        for (size_t i = 0; i < j; ++i)
        {
            Vicktor distance = bodies[i].position - bodies[j].position;
            u += -G * bodies[i].getMass() * bodies[j].getMass() / distance.module();
        }
    }
    return k + u;
}

Vicktor Simulation::centreOfMass() const
{
    Vicktor cmNumerator{0., 0.};
    double totalMass{0.};
    for (const auto &body : bodies)
    {
        cmNumerator += (body.position * body.getMass());
        totalMass += body.getMass();
    }
    return cmNumerator * (1 / totalMass);
}

double Simulation::totalMass() const
{
    return std::accumulate(bodies.begin(), bodies.end(), 0.0,
                           [](double sum, const Planet &p)
                           { return sum + p.getMass(); });
}

double Simulation::consAngularMomentum() const
{
    Vicktor cm = centreOfMass();
    double angularMomentum{0};
    for (const auto &body : bodies)
    {
        angularMomentum += body.getMass() * (((body.position.x - cm.x) * body.velocity.y) - (((body.position.y - cm.y) * body.velocity.x)));
    }
    return angularMomentum;
}

Vicktor Simulation::consMomentum() const
{

    return std::accumulate(bodies.begin(), bodies.end(), Vicktor{0., 0.}, [](Vicktor momentum, const Planet &p)
                           { return momentum + (p.velocity * p.getMass()); });
}

void Simulation::initAccelerations()
{
    for (size_t i = 0; i < bodies.size(); ++i)
    {
        bodies[i].acceleration = gravAcceleration(bodies, i, G);
    }
}

bool Simulation::explosiveCollision(long unsigned int i, long unsigned int j) const
{
    double relativeVelocity{0.};
    double escapeVelocity{0.};
    relativeVelocity = (bodies[i].velocity - bodies[j].velocity).module();
    escapeVelocity = sqrt(2 * G * (bodies[i].getMass() + bodies[j].getMass()) / cbrt(pow(bodies[i].getRadius(), 3) + pow(bodies[j].getRadius(), 3))); // Using new "merged" radius, assuming all the bodies have equal density.
    bool explosion{false};
    if (relativeVelocity >= escapeVelocity)
    {
        explosion = true;
    }
    return explosion;
}

void Simulation::step(double dt)
{
    // controllo collisioni:
    double totalMassUnmerged{totalMass()};
    bool collision = false;
    size_t numberBodies{bodies.size()};
    for (size_t j = 1; j < numberBodies; ++j)
    {
        if (bodies[j].getMass() == 0.0) // per evitare calcoli inutili toglie dal check i corpi già scontrati ma non ancora eliminati dal vector.
        {
            continue;
        }
        for (size_t i = 0; i < j; ++i)
        {
            if (bodies[i].getMass() == 0.0) // per evitare calcoli inutili toglie dal check i corpi già scontrati ma non ancora eliminati dal vector.
            {
                continue;
            }
            
            if (((bodies[i].position - bodies[j].position).module() <= (bodies[i].getRadius() + bodies[j].getRadius())) && explosiveCollision(i, j) == true )
            //urto con velRelativa>=velFuga, allora urto "elastico" in cui il corpo minore si frantuma in detriti, che vengono espulsi con la stessa velocità di impatto.
            {
                size_t major{0};
                size_t minor{0};
                if(bodies[i].getMass()>=bodies[j].getMass()){
                    major = i;
                    minor = j;
                }
                else{
                    major = j;
                    minor = i;
                }
                Vicktor posCm = ((bodies[i].position * bodies[i].getMass()) + (bodies[j].position * bodies[j].getMass())) * (1 / (bodies[i].getMass() + bodies[j].getMass()));
                Vicktor velCm = ((bodies[i].velocity * bodies[i].getMass()) + (bodies[j].velocity * bodies[j].getMass())) * (1 / (bodies[i].getMass() + bodies[j].getMass()));
                double placementRadius = bodies[i].getRadius() + bodies[j].getRadius();
                double debrisRadius = bodies[minor].getRadius()*cbrt(1. / 8.);
                double debrisVelocity = (bodies[i].velocity - bodies[j].velocity).module() * sqrt(bodies[major].getMass()/(bodies[major].getMass() + bodies[minor].getMass()));
                double buffer = pi / 4;
                for (int k = 0; k <= 7; ++k)
                {
                    bodies.emplace_back(bodies[minor].getMass() / 8,
                                        posCm.x + placementRadius * cos(buffer * k),
                                        posCm.y + placementRadius * sin(buffer * k),
                                        velCm.x + (debrisVelocity * cos(buffer * k)),
                                        velCm.y + (debrisVelocity * sin(buffer * k)),
                                        debrisRadius);
                }
                bodies[major].position = ((bodies[i].position * bodies[i].getMass()) + (bodies[j].position * bodies[j].getMass())) * (1 / (bodies[i].getMass() + bodies[j].getMass()));
                bodies[major].velocity = ((bodies[i].velocity * bodies[i].getMass()) + (bodies[j].velocity * bodies[j].getMass())) * (1 / (bodies[i].getMass() + bodies[j].getMass()));
                bodies[minor].setMass(0.);
                bodies[minor].setRadius(0.);
                collision = true;
            }
            else
            {
                if ((bodies[i].position - bodies[j].position).module() <= (bodies[i].getRadius() + bodies[j].getRadius()))
                //urto con velRelativa<velFuga, allora urto anelastico in cui i due corpi si uniscono.
                {
                    bodies[j].position = ((bodies[i].position * bodies[i].getMass()) + (bodies[j].position * bodies[j].getMass())) * (1 / (bodies[i].getMass() + bodies[j].getMass()));
                    bodies[j].velocity = ((bodies[i].velocity * bodies[i].getMass()) + (bodies[j].velocity * bodies[j].getMass())) * (1 / (bodies[i].getMass() + bodies[j].getMass()));
                    bodies[j].setMass(bodies[j].getMass() + bodies[i].getMass());
                    bodies[j].setRadius(cbrt(pow(bodies[j].getRadius(), 3) + pow(bodies[i].getRadius(), 3))); // New radius, assuming all the bodies have equal density.
                    bodies[i].setMass(0.);
                    bodies[i].setRadius(0.);
                    collision = true;
                }
            }
        }
    }
    bodies.erase(std::remove_if(bodies.begin(), bodies.end(),
                                [](const Planet &p)
                                { return p.getMass() == 0.0; }),
                 bodies.end());
    double epsilon = 1e-7 * totalMassUnmerged; // Tolleranza per gli errori di arrotondamento
    if (std::abs(totalMass() - totalMassUnmerged) > epsilon) // Si attiva solo se 3 o più corpi si toccano nello stesso momento, e l'algoritmo di "trasferimento massa" si romperebbe.
    {
        throw std::runtime_error("Ci sono state delle collisioni con più di due corpi in contemporanea, non calcolabili da questo programma.");
    }
    if (collision)
    {
        initAccelerations();
    }
    // Velocity verlet:
    for (auto &body : bodies)
    {
        body.position += (body.velocity * dt) + (body.acceleration * (0.5 * dt * dt));
    }
    for (size_t i = 0; i < bodies.size(); ++i)
    {
        Vicktor acc_old = bodies[i].acceleration;
        Vicktor acc_new = gravAcceleration(bodies, i, G);
        Vicktor acc_sum = acc_old + acc_new;
        bodies[i].velocity = bodies[i].velocity + (acc_sum * (0.5 * dt));
        bodies[i].acceleration = acc_new;
    }
    double e = consEnergy();
    double L = consAngularMomentum();
    double p = consMomentum().module();
    energyRange.update(e);
    angularMomentumRange.update(L);
    momentumRange.update(p);
}

}
