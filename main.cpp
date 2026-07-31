#include <iostream>
#include "N_bodies.hpp"
int main()
{
  Simulation prova;
  for (int i = 0; i < 10000; ++i)
  {
    prova.step(1);
  }
  return 0;
}
