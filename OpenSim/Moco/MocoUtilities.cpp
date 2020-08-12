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
#include <cstdarg>
#include <cstdio>
#include <iomanip>
#include <regex>

#include <simbody/internal/Visualizer_InputListener.h>

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

std::vector<std::string> OpenSim::createStateVariableNamesInSystemOrder(
        const Model& model) {
    std::unordered_map<int, int> yIndexMap;
    return createStateVariableNamesInSystemOrder(model, yIndexMap);
}
std::vector<std::string> OpenSim::createStateVariableNamesInSystemOrder(
        const Model& model, std::unordered_map<int, int>& yIndexMap) {
    yIndexMap.clear();
    std::vector<std::string> svNamesInSysOrder;
    auto s = model.getWorkingState();
    const auto svNames = model.getStateVariableNames();
    s.updY() = 0;
    std::vector<int> yIndices;
    for (int iy = 0; iy < s.getNY(); ++iy) {
        s.updY()[iy] = SimTK::NaN;
        const auto svValues = model.getStateVariableValues(s);
        for (int isv = 0; isv < svNames.size(); ++isv) {
            if (SimTK::isNaN(svValues[isv])) {
                svNamesInSysOrder.push_back(svNames[isv]);
                yIndices.emplace_back(iy);
                s.updY()[iy] = 0;
                break;
            }
        }
        if (SimTK::isNaN(s.updY()[iy])) {
            // If we reach here, this is an unused slot for a quaternion.
            s.updY()[iy] = 0;
        }
    }
    int count = 0;
    for (const auto& iy : yIndices) {
        yIndexMap.emplace(std::make_pair(count, iy));
        ++count;
    }
    SimTK_ASSERT2_ALWAYS((size_t)svNames.size() == svNamesInSysOrder.size(),
            "Expected to get %i state names but found %i.", svNames.size(),
            svNamesInSysOrder.size());
    return svNamesInSysOrder;
}

std::unordered_map<std::string, int> OpenSim::createSystemYIndexMap(
        const Model& model) {
    std::unordered_map<std::string, int> sysYIndices;
    auto s = model.getWorkingState();
    const auto svNames = model.getStateVariableNames();
    s.updY() = 0;
    for (int iy = 0; iy < s.getNY(); ++iy) {
        s.updY()[iy] = SimTK::NaN;
        const auto svValues = model.getStateVariableValues(s);
        for (int isv = 0; isv < svNames.size(); ++isv) {
            if (SimTK::isNaN(svValues[isv])) {
                sysYIndices[svNames[isv]] = iy;
                s.updY()[iy] = 0;
                break;
            }
        }
        if (SimTK::isNaN(s.updY()[iy])) {
            // If we reach here, this is an unused slot for a quaternion.
            s.updY()[iy] = 0;
        }
    }
    SimTK_ASSERT2_ALWAYS(svNames.size() == (int)sysYIndices.size(),
            "Expected to find %i state indices but found %i.", svNames.size(),
            sysYIndices.size());
    return sysYIndices;
}

std::vector<std::string> OpenSim::createControlNamesFromModel(
        const Model& model, std::vector<int>& modelControlIndices) {
    std::vector<std::string> controlNames;
    // Loop through all actuators and create control names. For scalar
    // actuators, use the actuator name for the control name. For non-scalar
    // actuators, use the actuator name with a control index appended for the
    // control name.
    // TODO update when OpenSim supports named controls.
    int count = 0;
    modelControlIndices.clear();
    for (const auto& actu : model.getComponentList<Actuator>()) {
        if (!actu.get_appliesForce()) {
            count += actu.numControls();
            continue;
        }
        std::string actuPath = actu.getAbsolutePathString();
        if (actu.numControls() == 1) {
            controlNames.push_back(actuPath);
            modelControlIndices.push_back(count);
            count++;
        } else {
            for (int i = 0; i < actu.numControls(); ++i) {
                controlNames.push_back(actuPath + "_" + std::to_string(i));
                modelControlIndices.push_back(count);
                count++;
            }
        }
    }

    return controlNames;
}
std::vector<std::string> OpenSim::createControlNamesFromModel(
        const Model& model) {
    std::vector<int> modelControlIndices;
    return createControlNamesFromModel(model, modelControlIndices);
}

std::unordered_map<std::string, int> OpenSim::createSystemControlIndexMap(
        const Model& model) {
    // We often assume that control indices in the state are in the same order
    // as the actuators in the model. However, the control indices are
    // allocated in the order in which addToSystem() is invoked (not
    // necessarily the order used by getComponentList()). So until we can be
    // absolutely sure that the controls are in the same order as actuators,
    // we can run the following check: in order, set an actuator's control
    // signal(s) to NaN and ensure the i-th control is NaN.
    // TODO update when OpenSim supports named controls.
    std::unordered_map<std::string, int> controlIndices;
    const SimTK::State state = model.getWorkingState();
    auto modelControls = model.updControls(state);
    int i = 0;
    for (const auto& actu : model.getComponentList<Actuator>()) {
        int nc = actu.numControls();
        SimTK::Vector origControls(nc);
        SimTK::Vector nan(nc, SimTK::NaN);
        actu.getControls(modelControls, origControls);
        actu.setControls(nan, modelControls);
        std::string actuPath = actu.getAbsolutePathString();
        for (int j = 0; j < nc; ++j) {
            OPENSIM_THROW_IF(!SimTK::isNaN(modelControls[i]), Exception,
                    "Internal error: actuators are not in the "
                    "expected order. Submit a bug report.");
            if (nc == 1) {
                controlIndices[actuPath] = i;
            } else {
                controlIndices[fmt::format("{}_{}", actuPath, j)] = i;
            }
            ++i;
        }
        actu.setControls(origControls, modelControls);
    }
    return controlIndices;
}

void OpenSim::checkOrderSystemControls(const Model& model) {
    createSystemControlIndexMap(model);
}

void OpenSim::checkRedundantLabels(std::vector<std::string> labels) {
    std::sort(labels.begin(), labels.end());
    auto it = std::adjacent_find(labels.begin(), labels.end());
    OPENSIM_THROW_IF(it != labels.end(), Exception,
            "Label '{}' appears more than once.", *it);
}

void OpenSim::checkLabelsMatchModelStates(const Model& model,
        const std::vector<std::string>& labels) {
    const auto modelStateNames = model.getStateVariableNames();
    for (const auto& label : labels) {
        OPENSIM_THROW_IF(modelStateNames.rfindIndex(label) == -1, Exception,
                "Expected the provided labels to match the model state "
                "names, but label {} does not correspond to any model "
                "state.",
                label);
    }
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
        model.realizeVelocity(state);
        SimTK::Vec3 forcesRight(0);
        SimTK::Vec3 torquesRight(0);
        // Loop through all Forces of the right side.
        for (const auto& smoothForce : forcePathsRightFoot) {
            Array<double> forceValues =
                    model.getComponent<Force>(smoothForce).getRecordValues(state);
            forcesRight += SimTK::Vec3(forceValues[0], forceValues[1],
                    forceValues[2]);
            torquesRight += SimTK::Vec3(forceValues[3], forceValues[4],
                    forceValues[5]);
        }
        SimTK::Vec3 forcesLeft(0);
        SimTK::Vec3 torquesLeft(0);
        // Loop through all Forces of the left side.
        for (const auto& smoothForce : forcePathsLeftFoot) {
            Array<double> forceValues =
                    model.getComponent<Force>(smoothForce).getRecordValues(state);
            forcesLeft += SimTK::Vec3(forceValues[0], forceValues[1],
                    forceValues[2]);
            torquesLeft += SimTK::Vec3(forceValues[3], forceValues[4],
                    forceValues[5]);
        }
        // Append row to table.
        SimTK::RowVector_<SimTK::Vec3> row(6);
        row(0) = forcesRight;
        row(1) = SimTK::Vec3(0);
        row(2) = forcesLeft;
        row(3) = SimTK::Vec3(0);
        row(4) = torquesRight;
        row(5) = torquesLeft;
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
