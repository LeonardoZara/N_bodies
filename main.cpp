#include <iostream>
#include <algorithm>
#include <SFML/Graphics.hpp>
#include <sstream>
#include <iomanip>
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
  double scale = 400.0 / 4.515e12;

  sf::RenderWindow window(sf::VideoMode(900, 900), "N-Body Simulation");
  window.setPosition(sf::Vector2i(50, 50));
  sf::RectangleShape fadeRectangle(sf::Vector2f(900.f, 900.f));
  fadeRectangle.setFillColor(sf::Color(0, 0, 0, 10)); // L'ultimo valore '10' è la trasparenza (alfa)

  std::vector<sf::Color> palette = {
      sf::Color::Red,
      sf::Color::Green,
      sf::Color::Blue,
      sf::Color::Yellow,
      sf::Color::Magenta,
      sf::Color::Cyan
    };

  /*Cose per la legenda:
  sf::Font font;
  if (!font.loadFromFile("font.ttf"))
  {
    std::cerr << "Errore: impossibile caricare il font.ttf!\n";
    // Puoi anche decidere di fare return EXIT_FAILURE; qui se il font è obbligatorio
  }
  sf::Text legendText;
  legendText.setFont(font);
  legendText.setCharacterSize(20);           // Dimensione del carattere in pixel
  legendText.setFillColor(sf::Color::White); // Colore del testo
  legendText.setPosition(10.f, 10.f);        // Posizione in alto a sinistra (x, y)
  */

  while (window.isOpen())
  {
    sf::Event event;
    while (window.pollEvent(event))
    {
      if (event.type == sf::Event::Closed)
        window.close();
    }
    for (int k = 0; k < 5; k++)
    {
      solar_system.step(dt);
    }
    // window.clear(sf::Color::Black);
    window.draw(fadeRectangle);

    for (size_t i = 0; i < solar_system.bodies.size(); ++i)
    {
      auto &body = solar_system.bodies[i];
      sf::Color bodyColor = palette[i % palette.size()];

      sf::CircleShape circle(6.f); // raggio grafico fisso
      circle.setFillColor(bodyColor);
      circle.setOrigin(6.f, 6.f); // centra il cerchio sul punto

      float screenX = 450 + body.position.x * scale;
      float screenY = 450 + body.position.y * scale;
      circle.setPosition(screenX, screenY);

      window.draw(circle);
    }

    // LEGENDA:
    /*double currentEnergy = solar_system.consEnergy();
    double currentMomentum = solar_system.consMomentum().module(solar_system.consMomentum());
    double currentAngMomentum = solar_system.consAngularMomentum();

    // Formatta il testo in modo pulito (notazione scientifica per numeri molto grandi/piccoli)
    std::ostringstream oss;
    oss << std::scientific << std::setprecision(4); // 4 cifre decimali
    oss << "Energia Meccanica: " << currentEnergy << " J\n";
    oss << "Quantita' di Moto: " << currentMomentum << " kg*m/s\n";
    oss << "Momento Angolare:  " << currentAngMomentum << " kg*m^2/s";

    // Assegna la stringa creata al testo e disegnalo
    legendText.setString(oss.str()); */

    // Sfondo legenda
    /*
    sf::RectangleShape legendBackground(sf::Vector2f(320.f, 110.f)); // Larghezza e altezza del box
    legendBackground.setFillColor(sf::Color::Yellow);          // Nero con trasparenza
    legendBackground.setPosition(5.f, 5.f);
    window.draw(legendBackground);
    */

    //window.draw(legendText);

    window.display();
  }
  //Cout vecchi, da capire cosa tenere e cosa no:
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