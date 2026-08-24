# Simulation of an N-Body system
Romani Sofia (matr. 0001215422), Zara Leonardo (matr. 0001242255), 
23 August 2026

## 1 Introduction
In celestial mechanics, two-body systems follow Keplerian orbits and have exact analytical solutions. The problem arises when more than two bodies begin to interact with each other: the differential equations that describe their motion are non-integrable, and it is not possible to express their positions as explicit functions of time. The only way to predict their motion is to approximate it using numerical methods. The goal of the program described here is to simulate and graphically visualize how these bodies behave in the presence of each other. To do so, we used the Velocity Verlet algorithm, a numerical method that solves differential equations through time discretization. By using a discrete small time step (dt), this method successfully maintains the conservation of energy, momentum, and angular momentum.

## 2 Implementation choices
The program has been divided into multiple header and source files to ensure a clean separation of concerns.

*   **Vector Mathematics (vector2d.hpp, vector2d.cpp):** we implemented a struct to create a 2D vector, with all the necessary mathematical methods, useful for the position, velocity and acceleration of each body.
*   **Core Physics Engine (n_bodies.hpp, n_bodies.cpp):** in these files we implemented the class Planet which stores all the useful information of each body; the struct MinMax Tracker to find the higher and lower value of energy, momentum and angular momentum with the goal of showing their conservation; the class Simulation where we implemented all the time functions needed to calculate the position, velocity, acceleration, momentum of the Planet types.
*   **User Interface (main.cpp):** handles all the graphic instructions for the SFML window rendering and the reading of the data text file (dati.txt).
*   **Testing:** we wrote some tests using Doctest to validate the mathematical operations, the orbit dynamics, and the collision logic.

We have also implemented a velocity-based collision system: If the relative velocity between two contacting bodies is below the escape velocity of the merged body ($v_{rel}<v_{esc}$), an inelastic collision occurs: the bodies merge into one while conserving mass and momentum. If the relative velocity meets or exceeds escape velocity ($v_{rel} \ge v_{esc}$), an explosive collision occurs: the lighter body shatters into a random even number of debris fragments ejected circularly and symmetrically with the same speed around the center of mass, while the larger body updates its state, conserving mass and momentum of the system.

### 2.1 Graphical Visualization and Interactivity
The graphical interface was built using SFML and we implemented the following features:

*   **Orbit trails:** std::deque containers store previous positions, rendered as a line strip with an opacity gradient (alpha) to create a fading tail effect.
*   **Logarithmic Radius Scaling:** Since the program deals with astronomical scales, we implemented a logarithmic function (radiusToRadius) that converts real astronomical radii into visually readable screen sizes, bounded between Mercury's radius (2 pixels) and the Sun's radius (10 pixels).
*   **Interactive Controls:** we included real-time adjustment of time step dt, so that the user can decide the accuracy of the simulation with the left/right arrow keys. It is also possible to change the integration steps per render frame (implemented as subSteps) with the up/down arrow keys. Finally we added a viewport zoom using mouse or trackpad inputs.
*   **Legend:** it is possible to see an overlaid text on the graphic window that displays real time values of the mechanical energy, momentum, angular momentum, body count and simulation setting as the dt and subStep per frame. After closing the graphic window, the overall minimum and maximum ranges for conserved quantities will be visible on the terminal as outputs.

## 3 Usage Instructions
To compile and execute the project on Ubuntu-based Linux systems the SFML library must be installed. The user can choose which astronomical configuration to display by removing the "#" from the relevant lines in the file dati.txt (each one describes a body) or writing one or more lines that contain the 6 space-separated numerical parameters in this order: Mass (kg), PositionX (m), PositionY (m), VelocityX (m/s), VelocityY (m/s), Radius (m). The parameters have physical limits such as the light speed for the velocity, a non negative mass and radius. Finally to compile and execute write the following command on the command line of Ubuntu terminal:

```bash
cmake -S . -B build -G"Ninja Multi-Config"
cmake --build build --config Debug
cmake --build build --config Debug --target test
cmake --build build --config Release
cmake --build build --config Release --target test
cd build/Release
./progetto_test
./progetto
```

## 4 Tests
Verification was done using unit and integration tests written with doctest in the test.cpp file. The goal was to cross check all of the following concepts:

*   **Vector Algebra:** the tests validate the vector addition, subtraction, scalar multiplication and module including the null vector case.
*   **Class Planet:** we tested the exactness of the Planet's methods such as getMass, getRadius, position and velocity.
*   **Class Simulation:** the goal of the tests is to verify the correctness of the conserved quantities for specific trajectories like constant velocity motion and elliptical orbit, where we put a 1% relative tolerance on the conservation of energy and angular momentum. Two more tests are dedicated to the totalMass and centreOfMass correctness.
*   **Collision Behavior:** we checked the merging of overlapping bodies, the conservation of the total mass and momentum both in inelastic merges and explosive collisions.

## 5 Use of generative artificial intelligence
Generative AI tools were used exclusively to assist in organizing unit test cases and to guide us through some concepts of the SFML library, such as the fading trails of the bodies and the loading of the font.

## 6 Conclusions
In standard C++ a double allocates 64 bits of memory and provides about 15 to 17 decimal digits of precision so the extremely large astronomical values can lose exact accuracy, that's why we needed to add a tolerance level (e) in the tests for the physical conservation laws and for the total mass of two merged bodies. For stable orbital systems, total mechanical energy and angular momentum remain conserved to a high degree of accuracy, proving the precision of Velocity Verlet integration.

However, the value of the time step (dt) strongly impacts the program's physical integrity. In general, with the Velocity Verlet algorithm, higher values of dt lead to a lower conservation of physical quantities, with oscillations in energy being proportional to $dt^{2}$. Furthermore, when testing collisions, dt should be kept as low as possible (possibly lower than 0.1 seconds) to exactly calculate the moment the bodies come into contact. If dt is too large, the program registers the collision when the bodies are overlapping, resulting in an artificially high relative velocity that could trigger explosive collisions with debris. Moreover, during this type of collisions, if an excessive dt is used, the false high relative velocity is transferred to the fragments, adding a bit of extra energy into the system. If dt is increased even further, bodies might completely pass through each other without registering any collision at all. The starting default dt in the program is set to 10 seconds, however, due to the listed reasons, if one wishes to accurately test collisions, during the simulation it is necessary to decrease dt as much as possible when bodies get close to each other to maintain the physical integrity of the simulation.
