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
#include "OpenSim/Common/ComponentPath.h"
#include "OpenSim/Common/SignalGenerator.h"
#include "OpenSim/Simulation/Control/InputController.h"
#include "OpenSim/Simulation/InverseDynamicsSolver.h"
#include "OpenSim/Simulation/Model/ExternalLoads.h"
#include "OpenSim/Common/STOFileAdapter.h"
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
        const MocoTrajectory& trajectory, Model& model, 
        std::string functionType) {
    // Get actuator names.
    model.initSystem();
    const auto& controlNames = trajectory.getControlNames();

    // Add a PrescribedController to control each actuator in the model, where 
    // the control functions are splined versions of the actuator controls from 
    // the OCP solution.
    const SimTK::Vector& time = trajectory.getTime();
    auto* controller = new PrescribedController();
    controller->setName("prescribed_controller");
    for (int i = 0; i < static_cast<int>(controlNames.size()); ++i) {
        const auto control = trajectory.getControl(controlNames[i]);
        std::unique_ptr<Function> function;
        if (functionType == "GCVSpline") {
            function = createFunction<GCVSpline>(time, control);
        } else if (functionType == "PiecewiseLinearFunction") {
            function = createFunction<PiecewiseLinearFunction>(time, control);
        } else {
            OPENSIM_THROW(
                    Exception, "Unexpected function type {}.", functionType);
        }
        const auto& actu = model.getComponent<Actuator>(controlNames[i]);
        controller->addActuator(actu);
        controller->prescribeControlForActuator(
                actu.getName(), *function);
    }
    model.addController(controller);

    // Add SignalGenerators to the model to control the Input controls 
    // associated with any InputControllers.
    for (auto& controller : model.updComponentList<InputController>()) {
        const auto& labels = controller.getInputControlLabels();
        for (const auto& label : labels) {
            std::string inputControlName = fmt::format("{}/{}", 
                    controller.getAbsolutePathString(), label);

            const auto inputControl = 
                    trajectory.getInputControl(inputControlName);
            std::unique_ptr<Function> function;
            if (functionType == "GCVSpline") {
                function = createFunction<GCVSpline>(time, inputControl);
            } else if (functionType == "PiecewiseLinearFunction") {
                function = createFunction<PiecewiseLinearFunction>(
                        time, inputControl);
            } else {
                OPENSIM_THROW(Exception, 
                        "Unexpected function type {}.", functionType);
            }
            auto* signal = new SignalGenerator();
            signal->setName(label);
            signal->set_function(*function);
            model.addComponent(signal);
            controller.connectInput_controls(signal->getOutput("signal"));
        }
    }
    model.finalizeConnections();
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

        // The simulated ground reaction forces and moments are just the 
        // opposite of the sum of forces and torques applied to the half-space.
        SimTK::Vec3 forcesRight = -halfSpaceForcesRight;
        SimTK::Vec3 forcesLeft = -halfSpaceForcesLeft;
        SimTK::Vec3 momentsRight = -halfSpaceTorquesRight;
        SimTK::Vec3 momentsLeft = -halfSpaceTorquesLeft;

        // The ground reaction moments should be equal to the moment caused by 
        // ground reaction force when applied at the center of pressure plus the 
        // vertical torque (see http://www.kwon3d.com/theory/grf/cop.html). 
        // Here, the vertical components of the forces and torques are in the 
        // y-direction (i.e., the y-component of the center of pressure is zero).
        // TODO: Support contact plane normals in any direction.

        // Compute centers of pressure.
        SimTK::Vec3 copRight(0);
        if (std::abs(forcesRight(1)) > SimTK::SignificantReal) {
            copRight(0) = momentsRight(2) / forcesRight(1);
            copRight(2) = -momentsRight(0) / forcesRight(1);
        }

        SimTK::Vec3 copLeft(0);
        if (std::abs(forcesLeft(1)) > SimTK::SignificantReal) {
            copLeft(0) = momentsLeft(2) / forcesLeft(1);
            copLeft(2) = -momentsLeft(0) / forcesLeft(1);
        }

        // Calculate the vertical torques.
        SimTK::Vec3 verticalTorqueRight(0);
        verticalTorqueRight(1) = momentsRight(1) - copRight(2) * forcesRight(0)
                + copRight(0) * forcesRight(2);
        SimTK::Vec3 verticalTorqueLeft(0);
        verticalTorqueLeft(1) = momentsLeft(1) - copLeft(2) * forcesLeft(0)
                + copLeft(0) * forcesLeft(2);

        // Append row to table.
        SimTK::RowVector_<SimTK::Vec3> row(6);
        row(0) = forcesRight;
        row(1) = copRight;
        row(2) = forcesLeft;
        row(3) = copLeft;
        row(4) = verticalTorqueRight;
        row(5) = verticalTorqueLeft;
        externalForcesTable.appendRow(state.getTime(), row);
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

TimeSeriesTable OpenSim::calcGeneralizedForces(Model model,
        const MocoTrajectory& trajectory,
        const std::vector<std::string>& forcePaths) {

    // Get the coordinates in multibody tree order, which is the order that the
    // inverse dynamics operator expects.
    model.initSystem();
    const auto& coordinates = model.getCoordinatesInMultibodyTreeOrder();

    // Compute the "known" udots from the trajectory.
    StatesTrajectory statesTraj = trajectory.exportToStatesTrajectory(model);
    TimeSeriesTable accelerationsTable =
            analyzeMocoTrajectory<double>(model, trajectory, {".*acceleration"});
    SimTK::Matrix udots(static_cast<int>(statesTraj.getSize()), 
            static_cast<int>(model.getNumCoordinates()));
    for (int j = 0; j < static_cast<int>(coordinates.size()); ++j) {
        const auto& coordinate = coordinates[j];
        udots.updCol(j) = accelerationsTable.getDependentColumn(
                fmt::format("{}|acceleration",
                            coordinate->getAbsolutePathString()));
    }

    // Get the multipliers from the trajectory. The inverse dynamics operator
    // will use these to apply the constraint forces to the model.
    TimeSeriesTable multipliersTable = trajectory.exportToMultipliersTable();    

    // Get the system indexes for specified forces.
    std::regex regex;
    SimTK::Array_<SimTK::ForceIndex> forceIndexes;
    for (const auto& forcePath : forcePaths) {
        regex = std::regex(forcePath);
        for (const auto& force : model.getComponentList<Force>()) {
            if (std::regex_match(force.getAbsolutePathString(), regex)) {
                auto it = std::find(forceIndexes.begin(), forceIndexes.end(),
                        force.getForceIndex());
                OPENSIM_THROW_IF(it != forceIndexes.end(), Exception,
                    "Expected unique force paths, but force at path " 
                    "'{}' was specified multiple times (possibly due to "
                    "specifying the full path and a regex pattern).", 
                    force.getAbsolutePathString());
            
                forceIndexes.push_back(force.getForceIndex());
            }
        }
    }

    // Add in the gravitational force index.
    const SimTK::Force::Gravity& gravity = model.getGravityForce();
    forceIndexes.push_back(gravity.getForceIndex());    

    // Compute the generalized forces.
    const auto& matter = model.getMatterSubsystem();
    const auto& forceSubsystem = model.getForceSubsystem();
    SimTK::Vector appliedMobilityForces(matter.getNumMobilities());
    SimTK::Vector_<SimTK::SpatialVec> appliedBodyForces(matter.getNumBodies());
    SimTK::Vector generalizedForces(model.getNumCoordinates());

    TimeSeriesTable generalizedForcesTable;
    for (int i = 0; i < static_cast<int>(statesTraj.getSize()); ++i) {
        const auto& state = statesTraj[i];
        generalizedForces.setToZero();

        model.realizeDynamics(state);
        appliedMobilityForces.setToZero();
        appliedBodyForces.setToZero();
        model.calcForceContributionsSum(state, forceIndexes, 
                appliedBodyForces, appliedMobilityForces);

        const auto& udot = ~udots.row(i);
        const auto& multipliers = multipliersTable.getRowAtIndex(i);
        matter.calcResidualForce(state, appliedMobilityForces, 
                appliedBodyForces, udot, ~multipliers, generalizedForces);

        generalizedForcesTable.appendRow(state.getTime(), ~generalizedForces);
    }

    // Set column labels.
    std::vector<std::string> labels;
    labels.reserve(coordinates.size());
    for (const auto& coordinate : coordinates) {
        std::string label = coordinate->getName();
        if (coordinate->getMotionType() == 
                Coordinate::MotionType::Rotational) {
            label += "_moment";
        } else if (coordinate->getMotionType() == 
                Coordinate::MotionType::Translational) {
            label += "_force";
        } else if (coordinate->getMotionType() == 
                Coordinate::MotionType::Coupled) {
            label += "_force";
        } else {
            OPENSIM_THROW(Exception,
                    "Expected coordinate '{}' to have Coordinate::MotionType "
                    "of Translational, Rotational, or Coupled, but it is "
                    "undefined.",
                    coordinate->getName());
        }
        labels.push_back(label);
    }
    generalizedForcesTable.setColumnLabels(labels);

    return generalizedForcesTable;
}
