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

#include <OpenSim/Common/Sine.h>
#include <OpenSim/Common/SignalGenerator.h>
#include <OpenSim/Common/Reporter.h>

#include "ComponentsForTesting.h"

using namespace OpenSim;
using namespace SimTK;


void testSignalGenerator() {
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
    SimTK_TEST_EQ((int)results.getNumRows(), numTimePoints);
    for (int i = 0; i < numTimePoints; ++i) {
        const double time = 0.1 * i;
        system.realize(s, Stage::Report);
        SimTK_TEST_EQ(results.getRowAtIndex(i)[0],
                amplitude * std::sin(omega * time + phase) + offset);
    }
}

int main() {

    SimTK_START_TEST("testSignalGenerator");
        SimTK_SUBTEST(testSignalGenerator);
    SimTK_END_TEST();
}
