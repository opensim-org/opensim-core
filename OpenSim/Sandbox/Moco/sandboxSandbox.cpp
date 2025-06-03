/* -------------------------------------------------------------------------- *
 * OpenSim Moco: sandboxSandbox.cpp                                           *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

// This file provides a way to easily prototype or test temporary snippets of
// code during development.

#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Actuators/ModelFactory.h>

using namespace OpenSim;

int main() {
    Model pendulum = ModelFactory::createNLinkPendulum(10);
    SimTK::State state = pendulum.initSystem();
    state.updY() = SimTK::Test::randVector(state.getNY());

    std::cout << "Using SimTK::TimeStepper directly: " << std::endl;
    const SimTK::MultibodySystem& system = pendulum.getMultibodySystem();
    SimTK::CPodesIntegrator integ(system);
    SimTK::TimeStepper ts(system, integ);
    ts.initialize(state);
    double cpuStart = SimTK::cpuTime();
    double realStart = SimTK::realTime();
    double finalTime = 10.0;
    ts.stepTo(finalTime);
    double cpu_time = SimTK::cpuTime()-cpuStart;
    double real_time = SimTK::realTime()-realStart;
    double realTimeFactor = finalTime/(real_time);
    std::cout << "cpu time:  "        << cpu_time << std::endl;
    std::cout << "real time: "        << real_time << std::endl;
    std::cout << "real time factor: " << realTimeFactor << std::endl;
    std::cout << std::endl;

    std::cout << "Using OpenSim::Manager: " << std::endl;
    Manager manager(pendulum);
    state = pendulum.initSystem();
    state.updTime() = 0.0;
    state.updY() = SimTK::Test::randVector(state.getNY());
    manager.setIntegratorMethod(Manager::IntegratorMethod::CPodes);
    manager.initialize(state);
    cpuStart = SimTK::cpuTime();
    realStart = SimTK::realTime();
    SimTK::State s = manager.integrate(finalTime);
    cpu_time = SimTK::cpuTime()-cpuStart;
    real_time = SimTK::realTime()-realStart;
    realTimeFactor = finalTime/(real_time);
    std::cout << "cpu time:  "        << cpu_time << std::endl;
    std::cout << "real time: "        << real_time << std::endl;
    std::cout << "real time factor: " << realTimeFactor << std::endl;

    state.updTime() = 0.0;
    state.updY() = SimTK::Test::randVector(state.getNY());
    manager.initialize(state);
    manager.integrate(20.0);


    // TimeSeriesTable statesTable = manager.getStatesTable();
    // std::cout << "States table:\n" << statesTable << std::endl;

    return EXIT_SUCCESS;
}
