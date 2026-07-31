#include <iostream>
#include "N_bodies.hpp"
int main()
{
  Simulation prova;
  prova.bodies.emplace_back(1.989e30, 0.0, 0.0, 0.0, 0.0);
  prova.bodies.emplace_back(5.972e24, 1.496, 0.0, 0.0, 29780.0);
  double dt = 3600.0;
  for (int i = 0; i < 8760; ++i)
  {
    prova.step(dt);
  }
  return 0;
}
