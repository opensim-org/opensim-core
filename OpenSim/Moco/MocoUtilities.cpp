/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoUtilities.cpp                                            *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia, Nicholas Bianco                             *
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

#include "MocoUtilities.h"

#include "MocoProblem.h"
#include "MocoTrajectory.h"
#include <regex>

#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Common/GCVSpline.h>
#include <OpenSim/Common/PiecewiseLinearFunction.h>
#include <OpenSim/Common/TableUtilities.h>
#include <OpenSim/Common/TimeSeriesTable.h>
#include <OpenSim/Simulation/Control/PrescribedController.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/StatesTrajectory.h>
#include <OpenSim/Simulation/StatesTrajectoryReporter.h>

using namespace OpenSim;

namespace {
template <typename FunctionType>
std::unique_ptr<Function> createFunction(
        const SimTK::Vector& x, const SimTK::Vector& y) {
    OPENSIM_THROW_IF(x.size() != y.size(), Exception, "x.size() != y.size()");
    return OpenSim::make_unique<FunctionType>(
            x.size(), x.getContiguousScalarData(), y.getContiguousScalarData());
}
template <>
std::unique_ptr<Function> createFunction<GCVSpline>(
        const SimTK::Vector& x, const SimTK::Vector& y) {
    OPENSIM_THROW_IF(x.size() != y.size(), Exception, "x.size() != y.size()");
    return OpenSim::make_unique<GCVSpline>(5, x.size(),
            x.getContiguousScalarData(), y.getContiguousScalarData());
}
} // anonymous namespace

void OpenSim::prescribeControlsToModel(
        const MocoTrajectory& trajectory, Model& model, std::string functionType) {
    // Get actuator names.
    model.initSystem();
    OpenSim::Array<std::string> actuNames;
    for (const auto& actu : model.getComponentList<Actuator>()) {
        actuNames.append(actu.getAbsolutePathString());
    }

    // Add prescribed controllers to actuators in the model, where the control
    // functions are splined versions of the actuator controls from the OCP
    // solution.
    const SimTK::Vector& time = trajectory.getTime();
    auto* controller = new PrescribedController();
    controller->setName("prescribed_controller");
    for (int i = 0; i < actuNames.size(); ++i) {
        const auto control = trajectory.getControl(actuNames[i]);
        std::unique_ptr<Function> function;
        if (functionType == "GCVSpline") {
            function = createFunction<GCVSpline>(time, control);
        } else if (functionType == "PiecewiseLinearFunction") {
            function = createFunction<PiecewiseLinearFunction>(time, control);
        } else {
            OPENSIM_THROW(
                    Exception, "Unexpected function type {}.", functionType);
        }
        const auto& actu = model.getComponent<Actuator>(actuNames[i]);
        controller->addActuator(actu);
        controller->prescribeControlForActuator(
                actu.getName(), function.release());
    }
    model.addController(controller);
}

MocoTrajectory OpenSim::simulateTrajectoryWithTimeStepping(
        const MocoTrajectory& trajectory, Model model,
        double integratorAccuracy) {

    prescribeControlsToModel(trajectory, model, "PiecewiseLinearFunction");

    // Add states reporter to the model.
    auto* statesRep = new StatesTrajectoryReporter();
    statesRep->setName("states_reporter");
    statesRep->set_report_time_interval(0.001);
    model.addComponent(statesRep);

    // Simulate!
    const SimTK::Vector& time = trajectory.getTime();
    SimTK::State state = model.initSystem();
    state.setTime(time[0]);
    Manager manager(model);

    // Set the initial state.
    {
        const auto& matrix = trajectory.getStatesTrajectory();
        TimeSeriesTable initialStateTable(
                std::vector<double>{trajectory.getInitialTime()},
                SimTK::Matrix(matrix.block(0, 0, 1, matrix.ncol())),
                trajectory.getStateNames());
        initialStateTable.addTableMetaData("inDegrees", std::string("no"));
        auto statesTraj = StatesTrajectory::createFromStatesTable(
                model, initialStateTable);
        state.setY(statesTraj.front().getY());
    }

    if (!SimTK::isNaN(integratorAccuracy)) {
        manager.getIntegrator().setAccuracy(integratorAccuracy);
    }
    manager.initialize(state);
    state = manager.integrate(time[time.size() - 1]);

    // Export results from states reporter to a TimeSeriesTable
    TimeSeriesTable states = statesRep->getStates().exportToTable(model);

    const auto& statesTimes = states.getIndependentColumn();
    SimTK::Vector timeVec((int)statesTimes.size(), statesTimes.data(), true);
    TimeSeriesTable controls =
            TableUtilities::resample<SimTK::Vector, PiecewiseLinearFunction>(
                    model.getControlsTable(), timeVec);
    // Fix column labels. (TODO: Not general.)
    auto labels = controls.getColumnLabels();
    for (auto& label : labels) { label = "/forceset/" + label; }
    controls.setColumnLabels(labels);

    // Create a MocoTrajectory to facilitate states trajectory comparison (with
    // dummy data for the multipliers, which we'll ignore).

    auto forwardSolution = MocoTrajectory(timeVec,
            {{"states", {states.getColumnLabels(), states.getMatrix()}},
                    {"controls", {controls.getColumnLabels(),
                                         controls.getMatrix()}}});

    return forwardSolution;
}

MocoTrajectory OpenSim::createPeriodicTrajectory(
        const MocoTrajectory& in, std::vector<std::string> addPatterns,
        std::vector<std::string> negatePatterns,
        std::vector<std::string> negateAndShiftPatterns,
        std::vector<std::pair<std::string, std::string>> symmetryPatterns) {

    const int oldN = in.getNumTimes();
    const int newN = 2 * oldN - 1;
    SimTK::Vector newTime(newN);
    newTime.updBlock(0, 0, oldN, 1) = in.getTime();
    newTime.updBlock(oldN, 0, oldN - 1, 1) =
            in.getTime().block(1, 0, oldN - 1, 1);
    newTime.updBlock(oldN, 0, oldN - 1, 1).updCol(0) +=
            in.getFinalTime() - in.getInitialTime();

    auto find = [](const std::vector<std::string>& v, const std::string& e) {
        return std::find(v.begin(), v.end(), e);
    };

    auto process = [&](std::string vartype,
                           const std::vector<std::string> names,
                           const SimTK::Matrix& oldTraj) -> SimTK::Matrix {
        SimTK::Matrix newTraj(newN, (int)names.size());
        for (int i = 0; i < (int)names.size(); ++i) {
            std::string name = names[i];
            newTraj.updBlock(0, i, oldN, 1) = oldTraj.col(i);
            bool matched = false;
            for (const auto& pattern : addPatterns) {
                const auto regex = std::regex(pattern);
                // regex_match() only returns true if the regex matches the
                // entire name.
                if (std::regex_match(name, regex)) {
                    matched = true;
                    const double& oldInit = oldTraj.getElt(0, i);
                    const double& oldFinal = oldTraj.getElt(oldN - 1, i);
                    newTraj.updBlock(oldN, i, oldN - 1, 1) =
                            oldTraj.block(1, i, oldN - 1, 1);
                    newTraj.updBlock(oldN, i, oldN - 1, 1).updCol(0) +=
                            oldFinal - oldInit;
                    break;
                }
            }

            for (const auto& pattern : negatePatterns) {
                const auto regex = std::regex(pattern);
                if (std::regex_match(name, regex)) {
                    matched = true;
                    const double& oldFinal = oldTraj.getElt(oldN - 1, i);
                    newTraj.updBlock(oldN, i, oldN - 1, 1) = SimTK::Matrix(
                            oldTraj.block(1, i, oldN - 1, 1).negate());
                    break;
                }
            }

            for (const auto& pattern : negateAndShiftPatterns) {
                const auto regex = std::regex(pattern);
                if (std::regex_match(name, regex)) {
                    matched = true;
                    const double& oldFinal = oldTraj.getElt(oldN - 1, i);
                    newTraj.updBlock(oldN, i, oldN - 1, 1) = SimTK::Matrix(
                            oldTraj.block(1, i, oldN - 1, 1).negate());
                    newTraj.updBlock(oldN, i, oldN - 1, 1).updCol(0) +=
                            2 * oldFinal;
                    break;
                }
            }

            for (const auto& pattern : symmetryPatterns) {
                const auto regex = std::regex(pattern.first);
                // regex_search() returns true if the regex matches any portion
                // of the name.
                if (std::regex_search(name, regex)) {
                    matched = true;
                    const auto opposite =
                            std::regex_replace(name, regex, pattern.second);
                    const auto it = find(names, opposite);
                    OPENSIM_THROW_IF(it == names.end(), Exception,
                                    "Could not find {} {}, which is supposed "
                                    "to be opposite of {}.",
                                    vartype, opposite, name);
                    const int iopp = (int)std::distance(names.cbegin(), it);
                    newTraj.updBlock(oldN, iopp, oldN - 1, 1) =
                            oldTraj.block(1, i, oldN - 1, 1);
                    break;
                }
            }

            if (!matched) {
                newTraj.updBlock(oldN, i, oldN - 1, 1) =
                        oldTraj.block(1, i, oldN - 1, 1);
            }
        }
        return newTraj;
    };

    SimTK::Matrix states =
            process("state", in.getStateNames(), in.getStatesTrajectory());

    SimTK::Matrix controls = process(
            "control", in.getControlNames(), in.getControlsTrajectory());

    SimTK::Matrix derivatives = process("derivative", in.getDerivativeNames(),
            in.getDerivativesTrajectory());

    return MocoTrajectory(newTime,
            {{"states", {in.getStateNames(), states}},
                    {"controls", {in.getControlNames(), controls}},
                    {"derivatives", {in.getDerivativeNames(), derivatives}}});
}


int OpenSim::getMocoParallelEnvironmentVariable() {
    const std::string varName = "OPENSIM_MOCO_PARALLEL";
    if (SimTK::Pathname::environmentVariableExists(varName)) {
        std::string parallel = SimTK::Pathname::getEnvironmentVariable(varName);
        int num = std::atoi(parallel.c_str());
        if (num < 0) {
            log_warn("OPENSIM_MOCO_PARALLEL environment variable set to "
                     "incorrect value '{}'; must be an integer >= 0. "
                     "Ignoring.", parallel);
        } else {
            return num;
        }
    }
    return -1;
}

TimeSeriesTable OpenSim::createExternalLoadsTableForGait(Model model,
        const StatesTrajectory& trajectory,
        const std::vector<std::string>& forcePathsRightFoot,
        const std::vector<std::string>& forcePathsLeftFoot) {
    model.initSystem();
    TimeSeriesTableVec3 externalForcesTable;
    int count = 0;
    for (const auto& state : trajectory) {
        model.realizeDynamics(state);
        SimTK::Vec3 sphereForcesRight(0);
        SimTK::Vec3 sphereTorquesRight(0);
        SimTK::Vec3 halfSpaceForcesRight(0);
        SimTK::Vec3 halfSpaceTorquesRight(0);
        // Loop through all Forces of the right side.
        for (const auto& smoothForce : forcePathsRightFoot) {
            Array<double> forceValues =
                    model.getComponent<Force>(smoothForce).getRecordValues(state);
            sphereForcesRight += SimTK::Vec3(forceValues[0], forceValues[1],
                    forceValues[2]);
            sphereTorquesRight += SimTK::Vec3(forceValues[3], forceValues[4],
                    forceValues[5]);
            halfSpaceForcesRight += SimTK::Vec3(forceValues[6], forceValues[7],
                    forceValues[8]);
            halfSpaceTorquesRight += SimTK::Vec3(forceValues[9], forceValues[10],
                    forceValues[11]);
        }
        SimTK::Vec3 sphereForcesLeft(0);
        SimTK::Vec3 sphereTorquesLeft(0);
        SimTK::Vec3 halfSpaceForcesLeft(0);
        SimTK::Vec3 halfSpaceTorquesLeft(0);
        // Loop through all Forces of the left side.
        for (const auto& smoothForce : forcePathsLeftFoot) {
            Array<double> forceValues =
                    model.getComponent<Force>(smoothForce).getRecordValues(state);
            sphereForcesLeft += SimTK::Vec3(forceValues[0], forceValues[1],
                    forceValues[2]);
            sphereTorquesLeft += SimTK::Vec3(forceValues[3], forceValues[4],
                    forceValues[5]);
            halfSpaceForcesLeft += SimTK::Vec3(forceValues[6], forceValues[7],
                    forceValues[8]);
            halfSpaceTorquesLeft += SimTK::Vec3(forceValues[9], forceValues[10],
                    forceValues[11]);
        }

        // Compute centers of pressure for both feet. We need to use the force
        // and torque information from the half space to compute the correct
        // locations.
        // TODO: Support contact plane normals in any direction.
        SimTK::Vec3 copRight(0);
        copRight(0) = halfSpaceTorquesRight(2) / halfSpaceForcesRight(1);
        copRight(2) = -halfSpaceTorquesRight(0) / halfSpaceForcesRight(1);

        SimTK::Vec3 copLeft(0);
        copLeft(0) = halfSpaceTorquesLeft(2) / halfSpaceForcesLeft(1);
        copLeft(2) = -halfSpaceTorquesLeft(0) / halfSpaceForcesLeft(1);

        // Append row to table.
        SimTK::RowVector_<SimTK::Vec3> row(6);
        row(0) = sphereForcesRight;
        row(1) = copRight;
        row(2) = sphereForcesLeft;
        row(3) = copLeft;
        row(4) = sphereTorquesRight;
        row(5) = sphereTorquesLeft;
        externalForcesTable.appendRow(state.getTime(), row);
        ++count;
    }
    // Create table.
    std::vector<std::string> labels{"ground_force_r_v", "ground_force_r_p",
            "ground_force_l_v", "ground_force_l_p", "ground_torque_r_",
            "ground_torque_l_"};
    externalForcesTable.setColumnLabels(labels);
    TimeSeriesTable externalForcesTableFlat =
            externalForcesTable.flatten({"x", "y", "z"});

    return externalForcesTableFlat;
}

TimeSeriesTable OpenSim::createExternalLoadsTableForGait(Model model,
        const MocoTrajectory& trajectory,
        const std::vector<std::string>& forcePathsRightFoot,
        const std::vector<std::string>& forcePathsLeftFoot) {
    StatesTrajectory statesTraj = trajectory.exportToStatesTrajectory(model);
    return createExternalLoadsTableForGait(std::move(model), statesTraj,
            forcePathsRightFoot, forcePathsLeftFoot);
}
