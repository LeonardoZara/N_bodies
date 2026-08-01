#include <iostream>
#include "N_bodies.hpp"
int main()
{
  Simulation solar_system;

  solar_system.bodies.emplace_back(1.989e30, 0.0, 0.0, 0.0, 0.0);          // sole
  solar_system.bodies.emplace_back(5.972e24, 1.496e11, 0.0, 0.0, 29780.0); // terra

  solar_system.initAccelerations();

  double dt = 3600.0;
  int n_steps = 24 * 365;

  for (int i = 0; i < n_steps; ++i)
  {
    solar_system.step(dt);
    if (i % (24 * 30) == 0)
    {
      std::cout << "t = " << (i * dt) / 86400.0 << " giorni | Terra: ("<< solar_system.bodies[1].position.x << ", "<< solar_system.bodies[1].position.y << ")\n";
      std::cout << "velocità sole: " << solar_system.bodies[0].velocity.module(solar_system.bodies[0].velocity) << '\n';
      std::cout << "velocità terra: " << solar_system.bodies[1].velocity.module(solar_system.bodies[1].velocity) << '\n';
    }
  }

  return 0;
}