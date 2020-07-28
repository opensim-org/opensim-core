/* -------------------------------------------------------------------------- *
 *                       OpenSim:  testFunctions.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Christopher Dembia                                              *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include "ComponentsForTesting.h"

#include <OpenSim/Common/CommonUtilities.h>
#include <OpenSim/Common/MultivariatePolynomialFunction.h>
#include <OpenSim/Common/Reporter.h>
#include <OpenSim/Common/SignalGenerator.h>
#include <OpenSim/Common/Sine.h>

#define CATCH_CONFIG_MAIN
#include <OpenSim/Auxiliary/catch.hpp>
#include <OpenSim/Common/PolynomialFunction.h>

using namespace OpenSim;
using namespace SimTK;


TEST_CASE("SignalGenerator") {
    // Feed a SignalGenerator's output into a TableReporter, and make sure the
    // reported value is correct.

    // Build the model.
    RootComponent world;
    auto* signalGen = new SignalGenerator();
    signalGen->setName("sinusoid");
    const double amplitude = 1.5;
    const double omega = 3.1;
    const double phase = 0.3;
    const double offset = 0.12345;
    signalGen->set_function(Sine(amplitude, omega, phase, offset));
    world.addComponent(signalGen);
    
    auto* reporter = new TableReporter();
    reporter->addToReport(signalGen->getOutput("signal"));
    world.addComponent(reporter);

    // Build the system.
    MultibodySystem system;
    world.buildUpSystem(system);
    State s = system.realizeTopology();
    system.realize(s, Stage::Model);

    // "Simulate."
    const int numTimePoints = 5;
    for (int i = 0; i < numTimePoints; ++i) {
        s.setTime(0.1 * i);
        system.realize(s, Stage::Report);
    }

    // Check that the SignalGenerator produced the correct values.
    const TimeSeriesTable_<Real>& results = reporter->getTable();
    REQUIRE(results.getNumRows() == numTimePoints);
    for (int i = 0; i < numTimePoints; ++i) {
        const double time = 0.1 * i;
        system.realize(s, Stage::Report);
        CHECK(results.getRowAtIndex(i)[0] ==
                Approx(amplitude * std::sin(omega * time + phase) + offset));
    }
}

TEST_CASE("Interpolate using PiecewiseLinearFunction") {
    SimTK::Vector x = createVector({0, 1});
    SimTK::Vector y = createVector({1, 0});
    SimTK::Vector newX = createVector({-1, 0.25, 0.75, 1.5});
    SimTK::Vector newY = OpenSim::interpolate(x, y, newX);

    SimTK_TEST(SimTK::isNaN(newY[0]));
    SimTK_TEST_EQ(newY[1], 0.75);
    SimTK_TEST_EQ(newY[2], 0.25);
    SimTK_TEST(SimTK::isNaN(newY[3]));
}

TEST_CASE("MultivariatePolynomialFunction") {
    SECTION("Input errors") {
        {
            MultivariatePolynomialFunction f(createVector({1}), -1, 1);
            CHECK_THROWS_WITH(f.calcValue(SimTK::Vector()),
                    Catch::Contains("Expected dimension"));
        }
        {
            MultivariatePolynomialFunction f(createVector({1}), 5, 1);
            CHECK_THROWS_WITH(f.calcValue(SimTK::Vector()),
                    Catch::Contains("Expected dimension"));
        }
        {
            MultivariatePolynomialFunction f(createVector({1}), 1, -1);
            CHECK_THROWS_WITH(f.calcValue(SimTK::Vector()),
                    Catch::Contains("Expected order"));
        }
        {
            SimTK::Vector coefficients(19);
            MultivariatePolynomialFunction f(coefficients, 3, 3);
            CHECK_THROWS_WITH(f.calcValue(createVector({1, 2, 3})),
                    Catch::Contains("Expected 20 coefficients but got 19"));
        }
        {
            SimTK::Vector coefficients(21);
            MultivariatePolynomialFunction f(coefficients, 3, 3);
            CHECK_THROWS_WITH(f.calcValue(createVector({1, 2, 3})),
                    Catch::Contains("Expected 20 coefficients but got 21"));
        }
    }
    SECTION("Consistent with PolynomialFunction") {
        PolynomialFunction univariate(createVector({2, 1, 3}));
        MultivariatePolynomialFunction multivariate(
                createVector({3, 1, 2}), 1, 2);
        SimTK::Vector x = createVector({0.338});
        CHECK(univariate.calcValue(x) == multivariate.calcValue(x));
    }
    SECTION("Test 3-dimensional 3rd order polynomial") {
        SimTK::Vector c = SimTK::Test::randVector(20);
        MultivariatePolynomialFunction f(c, 3, 3);
        SimTK::Vector input = createVector({0.3, 7.3, 0.8});
        const double x = input[0];
        const double y = input[1];
        const double z = input[2];
        double expected = c[0] + c[1]*z + c[2]*z*z + c[3] * z*z*z +
                          c[4]*y + c[5]*y*z + c[6]*y*z*z + c[7]*y*y +
                          c[8]*y*y*z + c[9]*y*y*y + c[10]*x + c[11]*x*z +
                          c[12]*x*z*z + c[13]*x*y + c[14]*x*y*z + c[15]*x*y*y +
                          c[16]*x*x + c[17]*x*x*z + c[18]*x*x*y + c[19]*x*x*x;
        CHECK(f.calcValue(input) == expected);
    }
    SECTION("Test 4-dimensional 1st order polynomial") {
        SimTK::Vector c = SimTK::Test::randVector(5);
        MultivariatePolynomialFunction f(c, 4, 1);
        SimTK::Vector input = createVector({0.3, 7.3, 0.8, 6.4});
        double expected = c[0] + c[1] * input[3] + c[2] * input[2] +
                          c[3] * input[1] + c[4] * input[0];
        CHECK(f.calcValue(input) == expected);
    }
}

TEST_CASE("solveBisection()") {

    auto calcResidual = [](const SimTK::Real& x) { return x - 3.78; };
    {
        const auto root = solveBisection(calcResidual, -5, 5, 1e-6);
        SimTK_TEST_EQ_TOL(root, 3.78, 1e-6);
        // Make sure the tolerance has an effect.
        SimTK_TEST_NOTEQ_TOL(root, 3.78, 1e-10);
    }
    {
        const auto root = solveBisection(calcResidual, -5, 5, 1e-10);
        SimTK_TEST_EQ_TOL(root, 3.78, 1e-10);
    }

    // Multiple roots.
    {
        auto parabola = [](const SimTK::Real& x) {
            return SimTK::square(x - 2.5);
        };
        REQUIRE_THROWS_AS(solveBisection(parabola, -5, 5), OpenSim::Exception);
    }
}
