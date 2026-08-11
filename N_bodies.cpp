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

void Simulation::step(double dt)
{
    // controllo collisioni:
    double totalMassUnmerged{totalMass()};
    bool merged = false;
    for (size_t j = 1; j < bodies.size(); ++j)
    {
        for (size_t i = 0; i < j; ++i)
        {
            if (bodies[i].getMass() == 0.0) // per evitare calcoli inutili toglie dal check i corpi già scontrati ma non ancora eliminati dal vector.
            {
                continue;
            }
            if ((bodies[i].position - bodies[j].position).module() <= (bodies[i].getRadius() + bodies[j].getRadius()))
            {
                bodies[j].position = ((bodies[i].position * bodies[i].getMass()) + (bodies[j].position * bodies[j].getMass())) * (1 / (bodies[i].getMass() + bodies[j].getMass()));
                bodies[j].velocity = ((bodies[i].velocity * bodies[i].getMass()) + (bodies[j].velocity * bodies[j].getMass())) * (1 / (bodies[i].getMass() + bodies[j].getMass()));
                bodies[j].setMass(bodies[j].getMass() + bodies[i].getMass());
                bodies[j].setRadius(cbrt(pow(bodies[j].getRadius(), 3) + pow(bodies[i].getRadius(), 3))); // New radius, assuming all the bodies have equal density.
                bodies[i].setMass(0.);
                merged = true;
            }
        }
    }
    bodies.erase(std::remove_if(bodies.begin(), bodies.end(),
                                [](const Planet &p)
                                { return p.getMass() == 0.0; }),
                 bodies.end());

    if (totalMass() != totalMassUnmerged) // Si attiva solo se 3 o più corpi si toccano nello stesso momento, e l'algoritmo di "trasferimento massa" si romperebbe.
    {
        throw std::runtime_error("Ci sono state delle collisioni con più di due corpi in contemporanea, non calcolabili da questo programma.");
    }
    if (merged)
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

Vicktor Simulation::lagrange(int i) const
// Questa funzione restituisce i punti di lagrange INIZIALI del sistema di due corpi inizialmente allineati sull'asse x.
{
    // bodies[0] e bodies[1] devono giacere sull'asse x, e bodies[0] deve avere massa maggiore.
    std::vector<Vicktor> lagPoints(5);
    double distance = (bodies[0].position - bodies[1].position).module();
    double x1{-bodies[1].getMass() * distance / (bodies[0].getMass() + bodies[1].getMass())};
    double x2{bodies[0].getMass() * distance / (bodies[0].getMass() + bodies[1].getMass())};
    double omega{sqrt(G * (bodies[0].getMass() + bodies[1].getMass()) / (pow(distance, 3)))};
    // L4=
    lagPoints[3].x = x1 + x2 / 2;
    lagPoints[3].y = sqrt(3) * distance / 2;
    // L5=
    lagPoints[4].x = x1 + x2 / 2;
    lagPoints[4].y = -sqrt(3) * distance / 2;
    // calcolare L1, L2, L3 è molto difficile, richiede algoritmo di stima soluzioni di equazioni di 5° grado. Vediamo se farlo oppuure no.

    return lagPoints[i] + centreOfMass();
}
