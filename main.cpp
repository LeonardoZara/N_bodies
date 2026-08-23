#include <iostream>
#include <algorithm>
#include <SFML/Graphics.hpp>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <deque>
#include "n_bodies.hpp"

int main()
{
  nb::Simulation sim;

  try
  {
    sim.loadFromFile("dati.txt");
  }
  catch (const std::exception &e)
  {
    std::cerr << "Errore nel caricamento dei corpi: " << e.what() << '\n';
    return EXIT_FAILURE;
  }

  sim.initAccelerations();

  sim.energyRange.update(sim.consEnergy());
  sim.angularMomentumRange.update(sim.consAngularMomentum());
  sim.momentumRange.update(sim.consMomentum().module());

  constexpr unsigned int windowWidth = 800;
  constexpr unsigned int windowHeight = 800;
  constexpr float screenMargin = 10.f;

  sf::RenderWindow window(sf::VideoMode(windowWidth, windowHeight), "N-Body Simulation");
  window.setPosition(sf::Vector2i(50, 50));

  std::vector<sf::Color> palette = {
      sf::Color::Red,
      sf::Color::Green,
      sf::Color::Blue,
      sf::Color::Yellow,
      sf::Color::Magenta,
      sf::Color::Cyan};

  sf::Font font;
  if (!font.loadFromFile("open_sans.ttf"))
  {
    std::cerr << "Errore: impossibile caricare il open_sans.ttf!\n";
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
  instructionsText.setPosition(10.f, 700.f);

  const size_t trailLength = 3000;
  std::vector<std::deque<sf::Vector2<double>>> trails(sim.numBodies());

  const double r_min = 2.4397e6; //  Mercury radius
  const double r_max = 6.9634e8; //  Sun radius
  const float graphic_min = 2.0f;
  const float graphic_max = 10.0f;
  const double logMin = std::log10(r_min);
  const double logMax = std::log10(r_max);

  auto radiusToRadius = [&](double radius) -> double
  {
    if (radius == 0.0)
    {
      return 0.0f;
    }
    if (radius <= r_min)
    {
      return graphic_min;
    }
    if (radius >= r_max)
    {
      return graphic_max;
    }
    double t = (std::log10(radius) - logMin) / (logMax - logMin);
    return graphic_min + static_cast<float>(t * (graphic_max - graphic_min));
  };

  // Setting the initial zoom based on the furthest body
  double maxInitialDist{0.};
  maxInitialDist = (*std::max_element(sim.begin(), sim.end(), [](const nb::Planet &a, const nb::Planet &b)
                                      { return a.position.module() < b.position.module(); }))
                       .position.module();
  double scale = 1.0;
  if (maxInitialDist > 0.0)
  {
    scale = ((windowWidth / 2.0) - screenMargin) / maxInitialDist;
  }

  double dt = 10.0;
  const double dtMin = 0.01;
  const double dtMax = 21600.0;
  int subSteps = 50;
  const int minSubSteps = 1;
  const int maxSubSteps = 100;

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

      // If a collision happens, the trails get rearranged to the new bodies
      if (trails.size() != sim.numBodies())
      {
        trails.assign(sim.numBodies(), std::deque<sf::Vector2<double>>());
      }
      window.clear(sf::Color::Black);

      for (size_t i = 0; i < sim.numBodies(); ++i)
      {
        auto &body = sim.getBody(i);
        sf::Color bodyColor = palette[i % palette.size()];

        // we save the body position for the trail
        trails[i].push_back(sf::Vector2<double>(body.position.x, body.position.y));
        if (trails[i].size() > trailLength)
        {
          trails[i].pop_front();
        }

        sf::VertexArray trailLine(sf::LineStrip, trails[i].size());
        for (size_t j = 0; j < trails[i].size(); ++j)
        {
          double trailScreenX = (windowWidth / 2) + trails[i][j].x * scale;
          double trailScreenY = (windowHeight / 2) + trails[i][j].y * scale;

          trailLine[j].position = sf::Vector2f(static_cast<float>(trailScreenX), static_cast<float>(trailScreenY));

          // progressive fading
          sf::Uint8 alpha = static_cast<sf::Uint8>((255 * j) / trails[i].size());
          trailLine[j].color = sf::Color(bodyColor.r, bodyColor.g, bodyColor.b, alpha);
        }
        window.draw(trailLine);

        double finalRadius = radiusToRadius(body.getRadius());

        sf::CircleShape circle(static_cast<float>(finalRadius));
        circle.setFillColor(bodyColor);
        circle.setOrigin(static_cast<float>(finalRadius), static_cast<float>(finalRadius));

        double screenX = (windowWidth / 2) + body.position.x * scale;
        double screenY = (windowHeight / 2) + body.position.y * scale;
        circle.setPosition(static_cast<float>(screenX), static_cast<float>(screenY));

        window.draw(circle);
      }

      std::ostringstream oss;
      oss << std::scientific << std::setprecision(8); // 8 significant figures
      oss << "Current Mechanical Energy: " << sim.consEnergy() << " J\n";
      oss << "Current Momentum: " << sim.consMomentum().module() << " kg*m/s\n";
      oss << "Current Angular Momentum:  " << sim.consAngularMomentum() << " kg*m^2/s\n";
      oss << "Current number of bodies: " << sim.numBodies() << '\n';

      legendText.setString(oss.str());

      window.draw(legendText);

      std::ostringstream oss2;
      oss2 << "To change the speed of the simulation press Up/Down arrows." << '\n';
      oss2 << "Current speed: " << subSteps << " iterations per frame" << '\n';
      oss2 << "To change the dt (time between each calculated iteration) press Left/Right arrows." << '\n';
      oss2 << "Current dt: " << dt << " seconds" << '\n';
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
