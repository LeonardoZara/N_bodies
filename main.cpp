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

  bool viewLagrange;
  if(solar_system.bodies.size()==2){
    std::cout<<"Vuoi visualizzare a schermo i punti di Lagrange del sistema? Scrivi 0 per non visualizzarli, 1 per visualizzarli." <<'\n';
    std::cin>>viewLagrange;
    if(viewLagrange==1){
      solar_system.bodies.emplace_back(1, solar_system.lagrange(3).x, solar_system.lagrange(3).y, 0., 0., 1);
      solar_system.bodies.emplace_back(1, solar_system.lagrange(4).x, solar_system.lagrange(4).y, 0., 0., 1);
    } else {
      if(viewLagrange!=0){
        std::cerr << "Inserire 0 oppure 1." << '\n';
      }
    }
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
  double scale = 360.0 / 4.515e12;

  sf::RenderWindow window(sf::VideoMode(800, 800), "N-Body Simulation");
  window.setPosition(sf::Vector2i(50, 50));

  //qui sarebbe per scie con la trasparenza, ma ora è con il vector
  //sf::RectangleShape fadeRectangle(sf::Vector2f(800.f, 800.f));
  //fadeRectangle.setFillColor(sf::Color(0, 0, 0, 7)); // L'ultimo valore '10' è la trasparenza (alfa)

  // roba per lo zoom
  sf::View view = window.getDefaultView();

  std::vector<sf::Color> palette = {
      sf::Color::Red,
      sf::Color::Green,
      sf::Color::Blue,
      sf::Color::Yellow,
      sf::Color::Magenta,
      sf::Color::Cyan};

  //Cose per la legenda:
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
  

  //aggiustare le scie per lo zoom: le facciamo con l'array invece che il fade rectangle
  const size_t MAX_TRAIL_LENGTH = 1000; // Lunghezza della scia (numero di punti memorizzati)
  std::vector<std::deque<sf::Vector2f>> trails(solar_system.bodies.size());

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
    }

    for (int k = 0; k < 15; k++)
    {
      solar_system.step(dt);
    }
    window.clear(sf::Color::Black);
    //window.draw(fadeRectangle); in questo caso le scie non si aggiusterebbero con lo zoom

    for (size_t i = 0; i < solar_system.bodies.size(); ++i)
    {
      auto &body = solar_system.bodies[i];
      sf::Color bodyColor = palette[i % palette.size()];


      //qui calcoliamo le scie
      // --- 1. GESTIONE DELLA SCIA ---
      // Salviamo la posizione FISICA (non i pixel) nella coda
      trails[i].push_back(sf::Vector2f(body.position.x, body.position.y));
      if (trails[i].size() > MAX_TRAIL_LENGTH) {
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

      //qui calcoliamo il raggio logaritmico per la grafica
      // 1. Definisci i limiti di massa (basati sui tuoi dati del Sistema Solare)
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
          double logMin  = std::log10(m_min);
          double logMax  = std::log10(m_max);
          
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
    double currentEnergy = solar_system.consEnergy();
    double currentMomentum = solar_system.consMomentum().module(solar_system.consMomentum());
    double currentAngMomentum = solar_system.consAngularMomentum();

    // Formatta il testo in modo pulito (notazione scientifica per numeri molto grandi/piccoli)
    std::ostringstream oss;
    oss << std::scientific << std::setprecision(4); // 4 cifre decimali
    oss << "Energia Meccanica: " << currentEnergy << " J\n";
    oss << "Quantita' di Moto: " << currentMomentum << " kg*m/s\n";
    oss << "Momento Angolare:  " << currentAngMomentum << " kg*m^2/s";

    // Assegna la stringa creata al testo e disegnalo
    legendText.setString(oss.str());

    window.draw(legendText);

    window.display();
  }
  // Cout vecchi, da capire cosa tenere e cosa no:
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