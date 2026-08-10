#include <iostream>
#include <algorithm>
#include <SFML/Graphics.hpp>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <deque>
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

  bool viewLagrange;
  if (sim.bodies.size() == 2)
  {
    std::cout << "Vuoi visualizzare a schermo i punti di Lagrange del sistema? Rispondi 0 per non visualizzarli, 1 per visualizzarli." << '\n';
    std::cin >> viewLagrange;
    if (viewLagrange == 1)
    {
      sim.bodies.emplace_back(1, sim.lagrange(3).x, sim.lagrange(3).y, 0., 0., 1);
      sim.bodies.emplace_back(1, sim.lagrange(4).x, sim.lagrange(4).y, 0., 0., 1);
    }
    else
    {
      if (viewLagrange != 0)
      {
        std::cerr << "Inserire 0 oppure 1." << '\n';
      }
    }
  }

  sim.initAccelerations();

  double initEnergy = sim.consEnergy();
  std::cout << "Energia meccanica iniziale: " << initEnergy << '\n';
  sim.energyRange.update(initEnergy);

  double initAngularMomentum = sim.consAngularMomentum();
  std::cout << "Momento angolare iniziale: " << initAngularMomentum << '\n';
  sim.angularMomentumRange.update(initAngularMomentum);

  double initMomentum = sim.consMomentum().module();
  std::cout << "Quantità di moto iniziale: " << initMomentum << '\n';
  sim.momentumRange.update(initMomentum);

  sf::RenderWindow window(sf::VideoMode(800, 800), "N-Body Simulation");
  window.setPosition(sf::Vector2i(50, 50));

  // roba per lo zoom
  sf::View view = window.getDefaultView();

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
  const size_t MAX_TRAIL_LENGTH = 1000; // Lunghezza della scia (numero di punti memorizzati)
  std::vector<std::deque<sf::Vector2f>> trails(sim.bodies.size());

  double dt = 3600.0;
  const double dtMin = 60.0;
  const double dtMax = 21600.0;
  double scale = 360.0 / 4.515e12;
  int subSteps = 15;
  const int minSubSteps = 1;
  const int maxSubSteps = 30;

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
      }
      if (event.type == sf::Event::KeyPressed)
      {
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
      trails[i].push_back(sf::Vector2f(body.position.x, body.position.y));
      if (trails[i].size() > MAX_TRAIL_LENGTH)
      {
        trails[i].pop_front();
      }

      // Disegniamo la scia applicando la variabile 'scale' in tempo reale (perfetto per lo zoom)
      sf::VertexArray trailLine(sf::LineStrip, trails[i].size());
      for (size_t j = 0; j < trails[i].size(); ++j)
      {
        float trailScreenX = 400 + trails[i][j].x * scale;
        float trailScreenY = 400 + trails[i][j].y * scale;

        trailLine[j].position = sf::Vector2f(trailScreenX, trailScreenY);

        // Trasparenza progressiva: più il punto è vecchio, più è trasparente
        sf::Uint8 alpha = static_cast<sf::Uint8>((255 * j) / trails[i].size());
        trailLine[j].color = sf::Color(bodyColor.r, bodyColor.g, bodyColor.b, alpha);
      }
      window.draw(trailLine);

      // qui calcoliamo il raggio logaritmico per la grafica
      //  1. Definisci i limiti di massa (basati sui tuoi dati del Sistema Solare)
      const double m_min = 3.3e23;   // Massa di Mercurio (limite inferiore)
      const double m_max = 1.989e30; // Massa del Sole (limite superiore)

      // 2. Definisci i limiti visivi in pixel
      const float r_min = 2.0f;  // Grandezza del "puntino" piccolo
      const float r_max = 10.0f; // Grandezza del cerchio massimo (es. Sole o Giove)

      // 3. Ottieni la massa del corpo corrente
      // (nota: ho usato getMass() basandomi sul tuo codice commentato.
      // Se nella tua struct è una variabile pubblica, usa semplicemente body.massa o simile)
      double currentMass = body.getMass();
      float finalRadius = r_min;

      // 4. Applica la logica delle soglie e del logaritmo
      if (currentMass <= m_min)
      {
        // Sotto o uguale al minimo: rimane un piccolo punto
        finalRadius = r_min;
      }
      else if (currentMass >= m_max)
      {
        // Sopra o uguale al massimo: raggiunge la grandezza massima
        finalRadius = r_max;
      }
      else
      {
        // Interpolazione logaritmica: calcoliamo quanto siamo distanti (in percentuale)
        // tra l'esponente di Mercurio e quello del Sole.
        double logMass = std::log10(currentMass);
        double logMin = std::log10(m_min);
        double logMax = std::log10(m_max);

        // t sarà un valore da 0.0 (vicino a m_min) a 1.0 (vicino a m_max)
        double t = (logMass - logMin) / (logMax - logMin);

        // Mappiamo la percentuale sui pixel
        finalRadius = r_min + static_cast<float>(t * (r_max - r_min));
      }

      sf::CircleShape circle(finalRadius); // se vogliamo rimetterlo fisso basta mettere (6.f)
      circle.setFillColor(bodyColor);
      circle.setOrigin(finalRadius, finalRadius); // centra il cerchio sul punto

      float screenX = 400 + body.position.x * scale;
      float screenY = 400 + body.position.y * scale;
      circle.setPosition(screenX, screenY);

      window.draw(circle);
    }

    // LEGENDA:
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
    oss2 << "Speed: " << subSteps << " iterations per frame" << '\n';
    oss2 << "To change the dt (time between each calculated iteration) press Left/Right arrows." << '\n';
    oss2 << "dt: " << dt / 60 << " minutes" << '\n';

    instructionsText.setString(oss2.str());

    window.draw(instructionsText);

    window.display();
  }

  std::cout << "L'energia oscilla tra " << sim.energyRange.max << " e " << sim.energyRange.min << '\n';
  std::cout << "Il momento angolare oscilla tra " << sim.angularMomentumRange.max << " e " << sim.angularMomentumRange.min << '\n';
  std::cout << "La quantità di moto oscilla tra " << sim.momentumRange.max << " e " << sim.momentumRange.min << '\n';
}
