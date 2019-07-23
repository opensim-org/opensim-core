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

#include "Components/ModelFactory.h"
#include "Components/PositionMotion.h"
#include "MocoCasADiSolver/MocoCasADiSolver.h"
#include "MocoGoal/MocoControlGoal.h"
#include "MocoGoal/MocoInitialActivationGoal.h"
#include "MocoProblem.h"
#include "MocoStudy.h"
#include "MocoUtilities.h"

#include <OpenSim/Tools/InverseDynamicsTool.h>

using namespace OpenSim;

void MocoInverse::constructProperties() {
    constructProperty_kinematics(TableProcessor());
    constructProperty_kinematics_allow_extra_columns(false);
    constructProperty_tolerance(1e-3);
    constructProperty_max_iterations();
    constructProperty_output_paths();
}

MocoStudy MocoInverse::initialize() const { return initializeInternal().first; }

std::pair<MocoStudy, TimeSeriesTable> MocoInverse::initializeInternal() const {
    using SimTK::Pathname;
    // Get the directory containing the setup file.
    std::string setupDir;
    {
        bool dontApplySearchPath;
        std::string fileName, extension;
        Pathname::deconstructPathname(getDocumentFileName(),
                dontApplySearchPath, setupDir, fileName, extension);
    }

    // Process inputs.
    // ----------------
    Model model = get_model().process();
    model.initSystem();

    TimeSeriesTable kinematics = get_kinematics().process(setupDir, &model);

    // Prescribe the kinematics.
    // -------------------------
    // allowMissingColumns = true: we only need kinematics.
    // allowExtraColumns = user-specified.
    // assemble = true: we must obey the kinematic constraints.
    auto statesTraj = StatesTrajectory::createFromStatesStorage(model,
            convertTableToStorage(kinematics), true,
            get_kinematics_allow_extra_columns(), true);

    auto posmot = PositionMotion::createFromStatesTrajectory(model, statesTraj);
    posmot->setName("position_motion");
    const auto* posmotPtr = posmot.get();
    model.addComponent(posmot.release());

    model.initSystem();

    // Set up the MocoProblem.
    // -----------------------

    MocoStudy moco;
    auto& problem = moco.updProblem();
    problem.setModelCopy(model);

    TimeInfo timeInfo;
    updateTimeInfo("kinematics", kinematics.getIndependentColumn().front(),
            kinematics.getIndependentColumn().back(), timeInfo);
    if (get_clip_time_range()) {
        timeInfo.initial += 1e-3;
        timeInfo.final -= 1e-3;
    }
    problem.setTimeBounds(timeInfo.initial, timeInfo.final);

    // TODO: Allow users to specify costs flexibly.
    problem.addGoal<MocoControlGoal>("excitation_effort");

    // Prevent "free" activation at the beginning of the motion.
    problem.addGoal<MocoInitialActivationGoal>("initial_activation");

    // Configure the MocoSolver.
    // -------------------------
    auto& solver = moco.initCasADiSolver();
    solver.set_dynamics_mode("implicit");
    OPENSIM_THROW_IF_FRMOBJ(get_tolerance() <= 0, Exception,
            format("Tolerance must be positive, but got %g.", get_tolerance()));
    solver.set_optim_convergence_tolerance(get_tolerance());
    solver.set_optim_constraint_tolerance(get_tolerance());
    // The sparsity detection works fine with DeGrooteFregly2016Muscle.
    solver.set_optim_sparsity_detection("random");
    // Forward is 3x faster than central.
    solver.set_optim_finite_difference_scheme("forward");
    solver.set_transcription_scheme("trapezoidal");

    solver.set_num_mesh_points(timeInfo.numMeshPoints);
    if (!getProperty_max_iterations().empty()) {
        solver.set_optim_max_iterations(get_max_iterations());
    }

    return std::make_pair(
            moco, posmotPtr->exportToTable(kinematics.getIndependentColumn()));
}

MocoInverseSolution MocoInverse::solve() const {
    std::pair<MocoStudy, TimeSeriesTable> init = initializeInternal();
    const auto& moco = init.first;

    MocoSolution mocoSolution = moco.solve().unseal();

    const auto& statesTrajTable = init.second;
    mocoSolution.insertStatesTrajectory(statesTrajTable);
    MocoInverseSolution solution;
    solution.setMocoSolution(mocoSolution);

    if (getProperty_output_paths().size()) {
        std::vector<std::string> outputPaths;
        for (int io = 0; io < getProperty_output_paths().size(); ++io) {
            outputPaths.push_back(get_output_paths(io));
        }
        solution.setOutputs(
                moco.analyze(solution.getMocoSolution(), outputPaths));
    }
    if (!mocoSolution.success()) {
        solution.m_mocoSolution.seal();
    }
    return solution;
}
