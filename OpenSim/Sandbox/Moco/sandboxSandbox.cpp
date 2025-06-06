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
#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Analyses/BodyKinematics.h>

using namespace OpenSim;

std::unique_ptr<Model> createSlidingMassModel() {
    auto model = std::make_unique<Model>();
    model->setName("sliding_mass");
    model->set_gravity(SimTK::Vec3(0, 0, 0));
    auto* body = new Body("body", 2.0, SimTK::Vec3(0), SimTK::Inertia(0));
    model->addBody(body);

    // Allows translation along x.
    auto* joint = new SliderJoint("slider", model->getGround(), *body);
    auto& coord = joint->updCoordinate(SliderJoint::Coord::TranslationX);
    coord.setName("position");
    model->addJoint(joint);

    auto* actu = new CoordinateActuator();
    actu->setCoordinate(&coord);
    actu->setName("actuator");
    actu->setOptimalForce(1);
    model->addForce(actu);

    PrescribedController* controller = new PrescribedController();
    controller->setName("controller");
    controller->addActuator(*actu);
    controller->prescribeControlForActuator(actu->getName(),
        Constant(1.7));
    model->addController(controller);

    BodyKinematics* bodyKinematics = new BodyKinematics();
    model->addAnalysis(bodyKinematics);

    body->attachGeometry(new Sphere(0.05));

    model->finalizeConnections();

    return model;
}

int main() {
    auto model = createSlidingMassModel();
    model->initSystem();

    double finalTime = 10.0;
    Manager manager(*model);
    manager.setReportStates(true);
    manager.setPerformAnalyses(true);
    manager.setWriteToStorage(true);
    SimTK::State state = model->initSystem();
    state.updTime() = 0.0;
    state.updY()[0] = SimTK::Pi/4; // Set initial angle to 45 degrees
    manager.setIntegratorMethod(Manager::IntegratorMethod::RungeKuttaMerson);
    manager.initialize(state);
    double cpuStart = SimTK::cpuTime();
    double realStart = SimTK::realTime();
    SimTK::State s = manager.integrate(finalTime/2.0);
    manager.integrate(finalTime);
    double cpu_time = SimTK::cpuTime()-cpuStart;
    double real_time = SimTK::realTime()-realStart;
    double realTimeFactor = finalTime/(real_time);
    std::cout << std::endl;
    std::cout << "cpu time:  "        << cpu_time << std::endl;
    std::cout << "real time: "        << real_time << std::endl;
    std::cout << "real time factor: " << realTimeFactor << std::endl;

    TimeSeriesTable statesTable = manager.getStatesTable();
    std::cout << "States table:" << std::endl;
    std::cout << "rows = " << statesTable.getNumRows() << std::endl;
    std::cout << "columns = " << statesTable.getNumColumns() << std::endl;
    auto statesTimes = statesTable.getIndependentColumn();
    std::cout << "times = ";
    for (const auto& time : statesTimes) {
        std::cout << time << " ";
    }
    std::cout << std::endl;

    TimeSeriesTable controlsTable = manager.getControlsTable();
    std::cout << "Controls table:" << std::endl;
    std::cout << "rows = " << controlsTable.getNumRows() << std::endl;
    std::cout << "columns = " << controlsTable.getNumColumns() << std::endl;
    auto controlsTimes = controlsTable.getIndependentColumn();
    std::cout << "times = ";
    for (const auto& time : controlsTimes) {
        std::cout << time << " ";
    }
    std::cout << std::endl;

    Storage statesStorage = manager.getStateStorage();
    std::cout << "States storage: " << statesStorage << std::endl;
    std::cout << "rows = " << statesStorage.getSize() << std::endl;
    std::cout << "columns = " << statesStorage.getColumnLabels().size() << std::endl;
    Array<double> statesStorageTimes;
    statesStorage.getTimeColumn(statesStorageTimes);
    std::cout << "times = ";
    for (int i = 0; i < statesStorageTimes.getSize(); ++i) {
        std::cout << statesStorageTimes[i] << " ";
    }
    std::cout << std::endl;

    StatesTrajectory statesTrajectory = manager.getStatesTrajectory();
    std::cout << "States trajectory: " << std::endl;
    std::cout << "size = " << statesTrajectory.getSize() << std::endl;
    std::cout << "times = ";    
    for (const auto& state : statesTrajectory) {
        std::cout << state.getTime() << " ";
    }
    std::cout << std::endl;

    model->getAnalysisSet().get(0).printResults("body_kin");
    TimeSeriesTable posTable("body_kin_BodyKinematics_acc_global.sto");
    std::cout << "Body kinematics table:" << std::endl;
    std::cout << "rows = " << posTable.getNumRows() << std::endl;
    std::cout << "columns = " << posTable.getNumColumns() << std::endl;
    auto posTimes = posTable.getIndependentColumn();
    std::cout << "times = ";
    for (const auto& time : posTimes) {
        std::cout << time << " ";
    }
    std::cout << std::endl;

    return EXIT_SUCCESS;
}
