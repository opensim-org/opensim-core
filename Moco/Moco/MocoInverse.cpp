/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoInverse.cpp                                              *
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

#include "MocoInverse.h"

#include "Components/PositionMotion.h"
#include "MocoCasADiSolver/MocoCasADiSolver.h"
#include "MocoCost/MocoControlCost.h"
#include "MocoCost/MocoSumSquaredStateCost.h"
#include "MocoProblem.h"
#include "MocoTool.h"
#include "MocoUtilities.h"

#include <OpenSim/Common/FileAdapter.h>
#include <OpenSim/Tools/InverseDynamicsTool.h>

using namespace OpenSim;

void MocoInverse::constructProperties() {

    constructProperty_initial_time();
    constructProperty_final_time();
    constructProperty_mesh_interval(0.02);
    constructProperty_kinematics_file("");
    constructProperty_kinematics_allow_extra_columns(false);
    constructProperty_lowpass_cutoff_frequency_for_kinematics(-1);
    constructProperty_external_loads_file("");
    constructProperty_ignore_activation_dynamics(false);
    constructProperty_ignore_tendon_compliance(false);
    constructProperty_create_reserve_actuators(-1);
    constructProperty_minimize_sum_squared_states(false);
    constructProperty_tolerance(1e-3);
    constructProperty_output_paths();
}

void MocoInverse::writeTableToFile(
        const TimeSeriesTable& table, const std::string& filepath) const {
    DataAdapter::InputTables tables = {{"table", &table}};
    FileAdapter::writeFile(tables, filepath);
}

MocoInverseSolution MocoInverse::solve() const {
    using SimTK::Pathname;
    // Get the directory containing the setup file.
    std::string setupDir;
    {
        bool dontApplySearchPath;
        std::string fileName, extension;
        Pathname::deconstructPathname(getDocumentFileName(),
                dontApplySearchPath, setupDir, fileName, extension);
    }

    Model model(m_model);
    model.finalizeFromProperties();
    for (auto& muscle : model.updComponentList<Muscle>()) {
        if (get_ignore_activation_dynamics()) {
            muscle.set_ignore_activation_dynamics(true);
        }
        if (get_ignore_tendon_compliance()) {
            muscle.set_ignore_tendon_compliance(true);
        }
    }

    MocoTool moco;
    auto& problem = moco.updProblem();

    // TODO: Move this elsewhere!
    for (const auto& muscle : model.getComponentList<Muscle>()) {
        if (!muscle.get_ignore_activation_dynamics()) {
            problem.setStateInfo(muscle.getAbsolutePathString() + "/activation",
                    // TODO: Use the muscle's minimum_activation.
                    {0.01, 1});
        }
        if (!muscle.get_ignore_tendon_compliance()) {
            // TODO shouldn't be necessary.
            problem.setStateInfo(
                    muscle.getAbsolutePathString() + "/norm_fiber_length",
                    {0.2, 1.8});
        }
    }

    InverseDynamicsTool idTool;
    if (!get_external_loads_file().empty()) {
        idTool.createExternalLoads(get_external_loads_file(), model);
    }

    model.initSystem();

    std::string kinematicsFilePath =
            Pathname::getAbsolutePathnameUsingSpecifiedWorkingDirectory(
                    setupDir, get_kinematics_file());

    FileAdapter::OutputTables tables =
            FileAdapter::readFile(kinematicsFilePath);
    // There should only be one table.
    OPENSIM_THROW_IF(tables.size() != 1, Exception,
            format("Expected the kinematics file '%s' to contain 1 table, but "
                   "it "
                   "contains %i tables.",
                    kinematicsFilePath, tables.size()));
    // Get the first table.
    auto* kinematicsRaw =
            dynamic_cast<TimeSeriesTable*>(tables.begin()->second.get());
    OPENSIM_THROW_IF(!kinematicsRaw, Exception,
            "Expected the provided kinematics file to contain a (scalar) "
            "TimeSeriesTable, but it contains a different type of table.");
    if (kinematicsRaw->hasTableMetaDataKey("inDegrees") &&
            kinematicsRaw->getTableMetaDataAsString("inDegrees") == "yes") {
        model.getSimbodyEngine().convertDegreesToRadians(*kinematicsRaw);
    }
    TimeSeriesTable kinematics;
    if (get_lowpass_cutoff_frequency_for_kinematics() != -1) {
        kinematics = filterLowpass(*kinematicsRaw,
                get_lowpass_cutoff_frequency_for_kinematics(), true);
    } else {
        kinematics = *kinematicsRaw;
    }

    // allowMissingColumns = true: we only need kinematics.
    // allowExtraColumns = user-specified.
    // assemble = true: we must obey the kinematic constraints.
    auto statesTraj = StatesTrajectory::createFromStatesStorage(model,
            convertTableToStorage(kinematics), true,
            get_kinematics_allow_extra_columns(), true);

    const auto coords = model.getCoordinatesInMultibodyTreeOrder();
    std::vector<std::string> coordSVNames;
    for (const auto& coord : coords) {
        coordSVNames.push_back(coord->getStateVariableNames()[0]);
    }
    auto posmot = PositionMotion::createFromTable(
            model, statesTraj.exportToTable(model, coordSVNames));
    posmot->setName("position_motion");
    model.addComponent(posmot.release());

    model.initSystem();
    if (get_create_reserve_actuators() != -1) {
        createReserveActuators(model, get_create_reserve_actuators());
    }

    problem.setModelCopy(model);

    const auto timeInfo = calcInitialAndFinalTimes(
            kinematicsRaw->getIndependentColumn(), {}, get_mesh_interval());
    // const double spaceForFiniteDiff = 1e-3;
    problem.setTimeBounds(timeInfo.initialTime, timeInfo.finalTime);

    // TODO: Allow users to specify costs flexibly.
    problem.addCost<MocoControlCost>("excitation_effort");
    if (get_minimize_sum_squared_states()) {
        problem.addCost<MocoSumSquaredStateCost>("activation_effort");
    }

    auto& solver = moco.initCasADiSolver();
    solver.set_dynamics_mode("implicit");
    if (getProperty_tolerance().size()) {
        OPENSIM_THROW_IF_FRMOBJ(get_tolerance() <= 0, Exception,
                format("Tolerance must be positive, but got %g.",
                        get_tolerance()));
        solver.set_optim_convergence_tolerance(get_tolerance());
        solver.set_optim_constraint_tolerance(get_tolerance());
    }
    // The sparsity detection works fine with DeGrooteFregly2016Muscle.
    solver.set_optim_sparsity_detection("random");
    // solver.set_optim_hessian_approximation("exact");
    // Forward is 3x faster than central.
    solver.set_optim_finite_difference_scheme("forward");
    if (model.getWorkingState().getNMultipliers()) {
        solver.set_transcription_scheme("hermite-simpson");
        solver.set_enforce_constraint_derivatives(true);
    }

    solver.set_num_mesh_points(timeInfo.numMeshPoints);
    MocoInverseSolution solution;
    solution.setMocoSolution(moco.solve().unseal());

    if (getProperty_output_paths().size()) {
        std::vector<std::string> outputPaths;
        for (int io = 0; io < getProperty_output_paths().size(); ++io) {
            outputPaths.push_back(get_output_paths(io));
        }
        solution.setOutputs(
                moco.analyze(solution.getMocoSolution(), outputPaths));
    }
    return solution;
}

MocoInverse::TimeInfo MocoInverse::calcInitialAndFinalTimes(
        const std::vector<double>& time0, const std::vector<double>& time1,
        const double& meshInterval) const {

    TimeInfo out;
    double initialTimeFromData = time0.front();
    double finalTimeFromData = time0.back();
    if (time1.size()) {
        initialTimeFromData = std::max(initialTimeFromData, time1.front());
        finalTimeFromData = std::min(finalTimeFromData, time1.back());
    }
    if (!getProperty_initial_time().empty()) {
        OPENSIM_THROW_IF_FRMOBJ(get_initial_time() < initialTimeFromData,
                Exception,
                format("Provided initial time of %g is less than what is "
                       "available from data, %g.",
                        get_initial_time(), initialTimeFromData));
        out.initialTime = get_initial_time();
    } else {
        out.initialTime = initialTimeFromData;
    }
    if (!getProperty_final_time().empty()) {
        OPENSIM_THROW_IF_FRMOBJ(get_final_time() > finalTimeFromData, Exception,
                format("Provided final time of %g is greater than what "
                       "is available from data, %g.",
                        get_final_time(), finalTimeFromData));
        out.finalTime = get_final_time();
    } else {
        out.finalTime = finalTimeFromData;
    }
    OPENSIM_THROW_IF_FRMOBJ(out.finalTime < out.initialTime, Exception,
            format("Initial time of %g is greater than final time of %g.",
                    out.initialTime, out.finalTime));

    // We do not want to end up with a lower mesh frequency than requested.
    out.numMeshPoints =
            (int)std::ceil((out.finalTime - out.initialTime) / (meshInterval)) +
            1;
    return out;
}
