#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include <fstream>
#include <cstdio>
#include <string>
#include <vector>
#include "doctest.h"
#include "vector2d.hpp"
#include "n_bodies.hpp"



TEST_CASE("Vector2d: basic operations")
{
    nb::Vector2d a{3.0, 4.0};
    nb::Vector2d b{1.0, 2.0};

    SUBCASE("sum"){
        nb::Vector2d c = a + b; 
        CHECK(c.x == doctest::Approx(4.0));
        CHECK(c.y == doctest::Approx(6.0));
    }
    SUBCASE("subtraction"){
        nb::Vector2d d = a - b;
        CHECK(d.x == doctest::Approx(2.0));
        CHECK(d.y == doctest::Approx(2.0));
    }
    SUBCASE("scalar multiplication"){
        nb::Vector2d c = a*2;
        CHECK(c.x == doctest::Approx(6.0));
        CHECK(c.y == doctest::Approx(8.0));
    }
    SUBCASE("module"){
        CHECK(a.module() == doctest::Approx(5.0));
    }
}
TEST_CASE("Vector2d: exceptions"){
        SUBCASE("null vector's module is zero"){
            nb::Vector2d null{0.0, 0.0};
            CHECK(null.module() == doctest::Approx(0.0));
        }
        SUBCASE("multiplication by zero"){
            nb::Vector2d a{2.0 ,4.0};
            nb::Vector2d c = a*0;
            CHECK(c.x == doctest::Approx(0.0));
            CHECK(c.y == doctest::Approx(0.0));

        }
        SUBCASE("subtracting a vector from itself") {
            nb::Vector2d a{3.0,4.0};
            nb::Vector2d c = a-a;
            CHECK(c.x == doctest::Approx(0.0));
            CHECK(c.y == doctest::Approx(0.0));
        }
}

TEST_CASE("Planet")
{
    nb::Planet p(5.0, 1.0, 2.0, 0.5, -0.5, 0.1);

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
    nb::Simulation sim;
    sim.addBody(10.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    sim.addBody(20.0, 5.0, 0.0, 0.0, 0.0, 1.0);
    sim.addBody(5.0, -5.0, 0.0, 0.0, 0.0, 1.0);

    CHECK(sim.totalMass() == doctest::Approx(35.0));
}

TEST_CASE("Simulation::centreOfMass - simmetric system")
{
    nb::Simulation sim;
    sim.addBody(1.0, -1.0, 0.0, 0.0, 0.0, 0.1);
    sim.addBody(1.0, 1.0, 0.0, 0.0, 0.0, 0.1);

    nb::Vector2d cm = sim.centreOfMass();
    CHECK(cm.x == doctest::Approx(0.0));
    CHECK(cm.y == doctest::Approx(0.0));
}

TEST_CASE("Simulation::consMomentum e consEnergy - still bodies")
{
    nb::Simulation sim;
    sim.addBody(10.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    sim.addBody(10.0, 5.0, 0.0, 0.0, 0.0, 1.0);

    nb::Vector2d p = sim.consMomentum();
    CHECK(p.x == doctest::Approx(0.0));
    CHECK(p.y == doctest::Approx(0.0));

    CHECK(sim.consEnergy() < 0.0); //no kinetic energy, just pontential (<0.0)
}



TEST_CASE("Simulation::step - one body moving with constant velocity")
{
    nb::Simulation sim;
    sim.addBody(1.0, 0.0, 0.0, 2.0, 3.0, 0.1);
    sim.initAccelerations(); // no other body. null acceleration

    sim.step(10.0); // dt = 10 s

    CHECK(sim.getBody(0).position.x == doctest::Approx(20.0));
    CHECK(sim.getBody(0).position.y == doctest::Approx(30.0));
    CHECK(sim.getBody(0).velocity.x == doctest::Approx(2.0));
    CHECK(sim.getBody(0).velocity.y == doctest::Approx(3.0));
}

TEST_CASE("Simulation::step - elliptical orbit")
{
    nb::Simulation sim;
    sim.addBody(1.989e30, 0.0, 0.0, 0.0, -0.154894, 6.96e8); //sun
    sim.addBody(5.972e24, 7.48e10, 0.0, 0.0, 51588.2, 6.371e6); //planet 
    
    sim.initAccelerations();

    double initialEnergy = sim.consEnergy();
    double initialAngMom = sim.consAngularMomentum();

    double dt = 3600.0; //we let the simulation go on 
    for (int i = 0; i < 24 * 30; ++i)
    {
        sim.step(dt);
    }

    double finalEnergy = sim.consEnergy();
    double finalAngMom = sim.consAngularMomentum();

    
    CHECK(finalEnergy == doctest::Approx(initialEnergy).epsilon(0.01));//tollerance error of 1% 
    CHECK(finalAngMom == doctest::Approx(initialAngMom).epsilon(0.01));
}

TEST_CASE("Simulation::step - merged bodies")
{
    nb::Simulation sim;
    sim.addBody(1.0, 0.0, 0.0, 0.0, 0.0, 5.0);
    sim.addBody(1.0, 1.0, 0.0, 0.0, 0.0, 5.0);
    sim.initAccelerations();

    REQUIRE(sim.numBodies() == 2);
    sim.step(1.0);

    CHECK(sim.numBodies() == 1); //bodies merged
    CHECK(sim.getBody(0).getMass() == doctest::Approx(2.0));
}

TEST_CASE("Simulation::step - collision with no debris")
{
    nb::Simulation sim;
    sim.addBody(1.0e20, -50.0, 0.0, 5.0, 0.0, 30.0);
    sim.addBody(1.0e20,  50.0, 0.0, -5.0, 0.0, 30.0);
    sim.initAccelerations();

    double initialMass = sim.totalMass();
    nb::Vector2d initialMomentum = sim.consMomentum();

    REQUIRE(sim.numBodies() == 2);
    for (int k = 0; k < 20000 && sim.numBodies() == 2; ++k)
    {
        sim.step(0.001);
    } 
    CHECK(sim.numBodies() == 1);
    
    // mass conservation
    CHECK(sim.getBody(0).getMass() == doctest::Approx(initialMass));
    
    // momentum conservation
    nb::Vector2d finalMomentum = sim.consMomentum();
    CHECK(finalMomentum.x == doctest::Approx(initialMomentum.x).epsilon(0.001));
    CHECK(finalMomentum.y == doctest::Approx(initialMomentum.y).epsilon(0.001));
    
    // final body must stop at the centre.
    CHECK(sim.getBody(0).velocity.module() == doctest::Approx(0.0));
}

TEST_CASE("Simulation::step - explosive collision with debris")
{
    nb::Simulation sim;
    sim.addBody(5.0e15, -20.0, 0.0, 500.0, 0.0, 10.0); 
    sim.addBody(1.0e15,  20.0, 0.0, -500.0, 0.0, 10.0);
    sim.initAccelerations();

    double initialMass = sim.totalMass();
    nb::Vector2d initialMomentum = sim.consMomentum();

    REQUIRE(sim.numBodies() == 2);
    
    for (int k = 0; k < 20000 && sim.numBodies() == 2; ++k)
    {
        sim.step(1e-4);
    } //we only need a small dt to make the collision happen

    CHECK(sim.numBodies() >= 3);
    
    // mass conservation
    CHECK(sim.totalMass() == doctest::Approx(initialMass).epsilon(0.0001));

    // momentum conservation
    nb::Vector2d finalMomentum = sim.consMomentum();
    CHECK(finalMomentum.x == doctest::Approx(initialMomentum.x).epsilon(0.001));
    CHECK(finalMomentum.y == doctest::Approx(initialMomentum.y).epsilon(0.001).scale(std::abs(initialMomentum.x)));
}