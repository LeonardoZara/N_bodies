#include <iostream>
#include <algorithm>
#include <SFML/Graphics.hpp>
#include <sstream>
#include <iomanip>
#include <cmath>
//#include <deque> //per generare le scie
#include "N_bodies.hpp"

int main()
{
  Simulation sim;

  try
  {
    sim.loadFromFile("dati.txt");
  }
  catch (const std::exception &e)
  {
    std::cerr << "Errore nel caricamento dei corpi: " << e.what() << '\n';
    return EXIT_FAILURE;
  }

  /*if (sim.bodies.size() == 2) // visualizzazione dei punti di lagrange, come se fossero corpi con massa 0.1 kg)
  {
    sim.bodies.emplace_back(0.1, sim.lagrange(3).x, sim.lagrange(3).y, 0., 0., 1);
    sim.bodies.emplace_back(0.1, sim.lagrange(4).x, sim.lagrange(4).y, 0., 0., 1);
  }
*/
  sim.initAccelerations();

  // Calculating initial energy and momentums.
  sim.energyRange.update(sim.consEnergy());
  sim.angularMomentumRange.update(sim.consAngularMomentum());
  sim.momentumRange.update(sim.consMomentum().module());

  constexpr unsigned int windowWidth = 800;
  constexpr unsigned int windowHeight = 800;
  constexpr float screenMargin = 10.f; // Margine in pixel per non far toccare inizialmente i pianeti sul bordo.

  sf::RenderWindow window(sf::VideoMode(windowWidth, windowHeight), "N-Body Simulation");
  window.setPosition(sf::Vector2i(50, 50));

  // roba per lo zoom
  // sf::View view = window.getDefaultView();

  std::vector<sf::Color> palette = {
      sf::Color::Red,
      sf::Color::Green,
      sf::Color::Blue,
      sf::Color::Yellow,
      sf::Color::Magenta,
      sf::Color::Cyan};

  // Cose per la legenda:
  sf::Font font;
  if (!font.loadFromFile("font.ttf"))
  {
    std::cerr << "Errore: impossibile caricare il font.ttf!\n";
    return EXIT_FAILURE;
  }

  sf::Text legendText;
  legendText.setFont(font);
  legendText.setCharacterSize(15);
  legendText.setFillColor(sf::Color::White);
  legendText.setPosition(10.f, 10.f);
  sf::Text instructionsText;
  instructionsText.setFont(font);
  instructionsText.setCharacterSize(15);
  instructionsText.setFillColor(sf::Color::White);
  instructionsText.setPosition(10.f, 650.f);

  // aggiustare le scie per lo zoom: le facciamo con l'array invece che il fade rectangle
  //const size_t trailLength = 1000; // Lunghezza della scia (numero di punti memorizzati)
  //std::vector<std::deque<sf::Vector2f>> trails(sim.bodies.size());

  // Mappatura massa -> raggio grafico.
  const double m_min = 3.3e23;   // Massa di Mercurio
  const double m_max = 1.989e30; // Massa del Sole
  const float r_min = 2.0f;
  const float r_max = 10.0f;
  const double logMin = std::log10(m_min);
  const double logMax = std::log10(m_max);

  auto massToRadius = [&](double mass) -> float
  {
    if (mass <= m_min)
    {
      return r_min;
    }
    if (mass >= m_max)
    {
      return r_max;
    }
    double t = (std::log10(mass) - logMin) / (logMax - logMin);
    return r_min + static_cast<float>(t * (r_max - r_min));
  };

  //Imposta la corretta scala di zoom inziale in base al corpo più lontano dall'origine.
  double maxInitialDist{0.};
  maxInitialDist = (*std::max_element(sim.bodies.begin(), sim.bodies.end(), [](const Planet &a, const Planet &b)
                                      { return a.position.module() < b.position.module(); }))
                       .position.module();
  double scale = 1.0;
  if (maxInitialDist > 0.0)
  {
    scale = ((windowWidth / 2.0) - screenMargin) / maxInitialDist;
  }


  double dt = 3600.0;
  const double dtMin = 0.01;
  const double dtMax = 21600.0;
  int subSteps = 15;
  const int minSubSteps = 1;
  const int maxSubSteps = 30;

  try
  {
  while (window.isOpen())
  {
    sf::Event event;
    while (window.pollEvent(event))
    {
      if (event.type == sf::Event::Closed)
      {
        window.close();
      }

      if (event.type == sf::Event::MouseWheelScrolled)
      {
        if (event.mouseWheelScroll.delta > 0)
        {
          scale *= 1.3; // zoom in
        }
        else
        {
          scale /= 1.3; // zoom out
        }
      }
      if (event.type == sf::Event::KeyPressed)
      {
        if (event.key.code == sf::Keyboard::Up)
        {
          subSteps = std::min(subSteps + 1, maxSubSteps);
        }
        if (event.key.code == sf::Keyboard::Down)
        {
          subSteps = std::max(subSteps - 1, minSubSteps);
        }
        if (event.key.code == sf::Keyboard::Right)
        {
          dt = std::min(dt * 1.2, dtMax);
        }
        if (event.key.code == sf::Keyboard::Left)
        {
          dt = std::max(dt / 1.2, dtMin);
        }
      }
    }
    for (int k = 0; k < subSteps; k++)
    {
      sim.step(dt);
    }
    window.clear(sf::Color::Black);

    for (size_t i = 0; i < sim.bodies.size(); ++i)
    {
      auto &body = sim.bodies[i];
      sf::Color bodyColor = palette[i % palette.size()];

      // qui calcoliamo le scie
      //  Salviamo la posizione fisica (non i pixel) nella coda
      //trails[i].push_back(sf::Vector2f(body.position.x, body.position.y));
      //if (trails[i].size() > trailLength)
      //{
      //  trails[i].pop_front();
      //}

      // Disegniamo la scia applicando la variabile 'scale' in tempo reale (perfetto per lo zoom)
      //sf::VertexArray trailLine(sf::LineStrip, trails[i].size());
      /*for (size_t j = 0; j < trails[i].size(); ++j)
      {
        double trailScreenX = (windowWidth / 2) + trails[i][j].x * scale;
        double trailScreenY = (windowHeight / 2) + trails[i][j].y * scale;

        trailLine[j].position = sf::Vector2f(trailScreenX, trailScreenY);

        // Trasparenza progressiva: più il punto è vecchio, più è trasparente
        sf::Uint8 alpha = static_cast<sf::Uint8>((255 * j) / trails[i].size());
        trailLine[j].color = sf::Color(bodyColor.r, bodyColor.g, bodyColor.b, alpha);
      }
      window.draw(trailLine);*/

      float finalRadius = massToRadius(body.getMass());

      sf::CircleShape circle(finalRadius); // se vogliamo rimetterlo fisso basta mettere (6.f)
      circle.setFillColor(bodyColor);
      circle.setOrigin(finalRadius, finalRadius); // centra il cerchio sul punto

      double screenX = (windowWidth/2) + body.position.x * scale;
      double screenY = (windowHeight/2) + body.position.y * scale;
      circle.setPosition(screenX, screenY);

      window.draw(circle);
    }

    // LEGENDE:
    double currentEnergy = sim.consEnergy();
    double currentMomentum = sim.consMomentum().module();
    double currentAngMomentum = sim.consAngularMomentum();

    // Formatta il testo in modo pulito (notazione scientifica per numeri molto grandi/piccoli)
    std::ostringstream oss;
    oss << std::scientific << std::setprecision(8); // 8 cifre decimali
    oss << "Current Mechanical Energy: " << currentEnergy << " J\n";
    oss << "Current Momentum: " << currentMomentum << " kg*m/s\n";
    oss << "Current Angular Momentum:  " << currentAngMomentum << " kg*m^2/s";

    // Assegna la stringa creata al testo e disegnalo
    legendText.setString(oss.str());

    window.draw(legendText);

    std::ostringstream oss2;
    oss2 << "To change the speed of the simulation press Up/Down arrows." << '\n';
    oss2 << "Current speed: " << subSteps << " iterations per frame" << '\n';
    oss2 << "To change the dt (time between each calculated iteration) press Left/Right arrows." << '\n';
    oss2 << "Current dt: " << dt / 60 << " minutes" << '\n';
    oss2 << "Use the mouse or the trackpad to zoom in/out." << '\n';

    instructionsText.setString(oss2.str());

    window.draw(instructionsText);

    window.display();
  }
 }
 catch (const std::exception &e)
 {
    std::cerr << "Errore nel calcolo delle collisioni. " << e.what() << '\n';
    return EXIT_FAILURE;
 }

  std::cout << "L'energia oscilla tra " << sim.energyRange.max << " e " << sim.energyRange.min << '\n';
  std::cout << "Il momento angolare oscilla tra " << sim.angularMomentumRange.max << " e " << sim.angularMomentumRange.min << '\n';
  std::cout << "La quantità di moto oscilla tra " << sim.momentumRange.max << " e " << sim.momentumRange.min << '\n';
}