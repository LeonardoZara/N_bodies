#include <iostream>
#include <algorithm>
#include <SFML/Graphics.hpp>
#include "N_bodies.hpp"

int main()
{
  Simulation solar_system;

  try
  {
    solar_system.loadFromFile("dati.txt");
  }
  catch (const std::exception &e)
  {
    std::cerr << "Errore nel caricamento dei corpi: " << e.what() << '\n';
    return EXIT_FAILURE;
  }
  solar_system.initAccelerations();

  double initEnergy = solar_system.consEnergy();
  std::cout << "Energia meccanica iniziale: " << initEnergy << '\n';
  solar_system.energiesHistory.push_back(initEnergy);

  double initAngularMomentum = solar_system.consAngularMomentum();
  std::cout << "Momento angolare iniziale: " << initAngularMomentum << '\n';
  solar_system.angularMomentumHistory.push_back(initAngularMomentum);

  double initMomentum = solar_system.consMomentum().module(solar_system.consMomentum());
  std::cout << "Quantità di moto iniziale: " << initMomentum << '\n';
  solar_system.momentumHistory.push_back(initMomentum);

  double dt = 3600.0;
  double scale = 350.0 / 1.496e11;

  sf::RenderWindow window(sf::VideoMode(800, 600), "N-Body Simulation");

  while (window.isOpen())
  {
    sf::Event event;
    while (window.pollEvent(event))
    {
      if (event.type == sf::Event::Closed)
        window.close();
    }
    for (int k = 0; k < 10; k++)
    {
      solar_system.step(dt);
    }
    window.clear(sf::Color::Black);

    for (auto &body : solar_system.bodies)
    {
      sf::CircleShape circle(6.f); // raggio grafico fisso
      circle.setFillColor(sf::Color::White);
      circle.setOrigin(6.f, 6.f); // centra il cerchio sul punto

      float screenX = 400 + body.position.x * scale;
      float screenY = 300 + body.position.y * scale;
      circle.setPosition(screenX, screenY);

      window.draw(circle);
    }

    window.display();
  }
  /*for (int i = 0; i < n_steps; ++i) Questo ciclo senza sfml NON eliminiamolo che poi vediamo come implementare i cout.
  {
    solar_system.step(dt);
    if (i % (24 * 30) == 0)
    {
      std::cout << "t = " << (i * dt) / 86400.0 << " giorni | Terra: (" << solar_system.bodies[1].position.x << ", " << solar_system.bodies[1].position.y << ")\n";
      std::cout << "velocità sole: " << solar_system.bodies[0].velocity.module(solar_system.bodies[0].velocity) << '\n';
      std::cout << "velocità terra: " << solar_system.bodies[1].velocity.module(solar_system.bodies[1].velocity) << '\n';
      std::cout << "t = " << (i * dt) / 86400.0 << " giorni | numero corpi: " << solar_system.bodies.size() << '\n';

      if (solar_system.bodies.size() > 1)
      {
        std::cout << "Terra: (" << solar_system.bodies[1].position.x << ", " << solar_system.bodies[1].position.y << ")\n";
        std::cout << "velocità sole: " << solar_system.bodies[0].velocity.module(solar_system.bodies[0].velocity) << '\n';
        std::cout << "velocità terra: " << solar_system.bodies[1].velocity.module(solar_system.bodies[1].velocity) << '\n';
      }
      else
      {
        std::cout << "Fusione avvenuta -> massa corpo unico: " << solar_system.bodies[0].getMass() << '\n';
      }
      std::cout << "Energia meccanica: " << solar_system.consEnergy() << '\n';
    }
  }*/
  double maxEnergy = *std::max_element(std::begin(solar_system.energiesHistory), std::end(solar_system.energiesHistory));
  double minEnergy = *std::min_element(std::begin(solar_system.energiesHistory), std::end(solar_system.energiesHistory));
  std::cout << "L'energia oscilla tra " << maxEnergy << " e " << minEnergy << '\n';

  double maxAngularMomentum = *std::max_element(std::begin(solar_system.angularMomentumHistory), std::end(solar_system.angularMomentumHistory));
  double minAngularMomentum = *std::min_element(std::begin(solar_system.angularMomentumHistory), std::end(solar_system.angularMomentumHistory));
  std::cout << "Il momento angolare oscilla tra " << maxAngularMomentum << " e " << minAngularMomentum << '\n';

  double maxMomentum = *std::max_element(std::begin(solar_system.momentumHistory), std::end(solar_system.momentumHistory));
  double minMomentum = *std::min_element(std::begin(solar_system.momentumHistory), std::end(solar_system.momentumHistory));
  std::cout << "la quantità di moto oscilla tra " << maxMomentum << " e " << minMomentum << '\n';
}