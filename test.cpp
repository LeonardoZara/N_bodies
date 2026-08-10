//CIAO QUI FAREMO TANTI BEI TEST
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "N_bodies.hpp"
#include "vicktor.hpp"


TEST_CASE("Vicktor: basic operations")
{
    Vicktor a{3.0, 4.0};
    Vicktor b{1.0, 2.0};

    SUBCASE("sum"){
        Vicktor c = a + b; 
        CHECK(c.x == doctest::Approx(4.0));
        CHECK(c.y == doctest::Approx(6.0));
    }
    SUBCASE("subtraction"){
        Vicktor d = a - b;
        CHECK(d.x == doctest::Approx(2.0));
        CHECK(d.y == doctest::Approx(2.0));
    }
    SUBCASE("scalar multiplication"){
        Vicktor c = a*2;
        CHECK(c.x == doctest::Approx(6.0));
        CHECK(c.y == doctest::Approx(8.0));
    }
    SUBCASE("module"){
        CHECK(a.module() == doctest::Approx(5.0));
    }
}
TEST_CASE("Vicktor: exceptions"){
        SUBCASE("null vector's module is zero"){
            Vicktor null{0.0, 0.0};
            CHECK(null.module() == doctest::Approx(0.0));
        }
        SUBCASE("multiplication by zero"){
            Vicktor a{2.0 ,4.0};
            Vicktor c = a*0;
            CHECK(c.x == doctest::Approx(0.0));
            CHECK(c.y == doctest::Approx(0.0));

        }
        SUBCASE("subtracting a vector from itself") {
            Vicktor a{3.0,4.0};
            Vicktor c = a-a;
            CHECK(c.x == doctest::Approx(0.0));
            CHECK(c.y == doctest::Approx(0.0));
        }
}

TEST_CASE("Planet")
{
    Planet p(5.0, 1.0, 2.0, 0.5, -0.5, 0.1);

    CHECK(p.getMass() == doctest::Approx(5.0));
    CHECK(p.getRadius() == doctest::Approx(0.1));
    CHECK(p.position.x == doctest::Approx(1.0));
    CHECK(p.position.y == doctest::Approx(2.0));
    CHECK(p.velocity.x == doctest::Approx(0.5));
    CHECK(p.velocity.y == doctest::Approx(-0.5));

    //default acceleration 
    CHECK(p.acceleration.x == doctest::Approx(0.0));
    CHECK(p.acceleration.y == doctest::Approx(0.0));
}

TEST_CASE("Simulation::totalMass")
{
    Simulation sim;
    sim.bodies.emplace_back(10.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    sim.bodies.emplace_back(20.0, 5.0, 0.0, 0.0, 0.0, 1.0);
    sim.bodies.emplace_back(5.0, -5.0, 0.0, 0.0, 0.0, 1.0);

    CHECK(sim.totalMass() == doctest::Approx(35.0));
}

TEST_CASE("Simulation::centreOfMass - simmetric system")
{
    Simulation sim;
    sim.bodies.emplace_back(1.0, -1.0, 0.0, 0.0, 0.0, 0.1);
    sim.bodies.emplace_back(1.0, 1.0, 0.0, 0.0, 0.0, 0.1);

    Vicktor cm = sim.centreOfMass();
    CHECK(cm.x == doctest::Approx(0.0));
    CHECK(cm.y == doctest::Approx(0.0));
}

TEST_CASE("Simulation::consMomentum e consEnergy - still bodies")
{
    Simulation sim;
    sim.bodies.emplace_back(10.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    sim.bodies.emplace_back(10.0, 5.0, 0.0, 0.0, 0.0, 1.0);

    Vicktor p = sim.consMomentum();
    CHECK(p.x == doctest::Approx(0.0));
    CHECK(p.y == doctest::Approx(0.0));

    CHECK(sim.consEnergy() < 0.0); //no kinetic energy, just pontential (<0.0)
}



TEST_CASE("Simulation::step - one body moving with constant velocity")
{
    Simulation sim;
    sim.bodies.emplace_back(1.0, 0.0, 0.0, 2.0, 3.0, 0.1);
    sim.initAccelerations(); // no other body. null acceleration

    sim.step(10.0); // dt = 10 s

    CHECK(sim.bodies[0].position.x == doctest::Approx(20.0));
    CHECK(sim.bodies[0].position.y == doctest::Approx(30.0));
    CHECK(sim.bodies[0].velocity.x == doctest::Approx(2.0));
    CHECK(sim.bodies[0].velocity.y == doctest::Approx(3.0));
}

TEST_CASE("Simulation::step - elliptical orbit")
{
    Simulation sim;
    // Sole
    sim.bodies.emplace_back(1.989e30, 0.0, 0.0, 0.0, -0.154894, 6.96e8);
    // Pianeta in orbita ellittica
    sim.bodies.emplace_back(5.972e24, 7.48e10, 0.0, 0.0, 51588.2, 6.371e6);
    
    sim.initAccelerations();

    // 1. Salviamo i valori fisici iniziali
    double initialEnergy = sim.consEnergy();
    double initialAngMom = sim.consAngularMomentum();

    // 2. Facciamo avanzare la simulazione per un mese (30 giorni)
    // Usiamo un dt di 3600 secondi (1 ora)
    double dt = 3600.0;
    for (int i = 0; i < 24 * 30; ++i)
    {
        sim.step(dt);
    }

    // 3. Calcoliamo i valori fisici finali
    double finalEnergy = sim.consEnergy();
    double finalAngMom = sim.consAngularMomentum();

    // 4. Verifichiamo la conservazione (con un margine di tolleranza)
    // Nota: Lavorando con numeri astronomici giganteschi (es. 10^30), 
    // i double perdono un po' di precisione. Usiamo epsilon() per dare
    // una tolleranza relativa dell'1% (0.01) sui controlli.
    CHECK(finalEnergy == doctest::Approx(initialEnergy).epsilon(0.01));
    CHECK(finalAngMom == doctest::Approx(initialAngMom).epsilon(0.01));
}

TEST_CASE("Simulation::step - merged bodies")
{
    Simulation sim;
    sim.bodies.emplace_back(1.0, 0.0, 0.0, 0.0, 0.0, 5.0);
    sim.bodies.emplace_back(1.0, 1.0, 0.0, 0.0, 0.0, 5.0);
    sim.initAccelerations();

    REQUIRE(sim.bodies.size() == 2);
    sim.step(1.0);

    CHECK(sim.bodies.size() == 1);// bodies merged
    CHECK(sim.bodies[0].getMass() == doctest::Approx(2.0));
}




