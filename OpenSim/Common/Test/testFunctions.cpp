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
#include <OpenSim/Common/Reporter.h>
#include <OpenSim/Common/SignalGenerator.h>
#include <OpenSim/Common/Sine.h>

#define CATCH_CONFIG_MAIN
#include <OpenSim/Auxiliary/catch.hpp>

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
