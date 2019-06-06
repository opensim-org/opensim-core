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
#include "MocoCost/MocoControlCost.h"
#include "MocoProblem.h"
#include "MocoStudy.h"
#include "MocoUtilities.h"

#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Tools/InverseDynamicsTool.h>

using namespace OpenSim;

void MocoInverse::constructProperties() {
    constructProperty_kinematics(TableProcessor());
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

    Model model = get_model().process();

    MocoStudy moco;
    auto& problem = moco.updProblem();

    model.initSystem();

    TimeSeriesTable kinematics = get_kinematics().process(setupDir, &model);

    // allowMissingColumns = true: we only need kinematics.
    // allowExtraColumns = false: user might have made an error.
    // assemble = true: we must obey the kinematic constraints.
    auto statesTraj = StatesTrajectory::createFromStatesStorage(
            model, convertTableToStorage(kinematics), true, false, true);

    auto posmot = PositionMotion::createFromStatesTrajectory(model, statesTraj);
    posmot->setName("position_motion");
    model.addComponent(posmot.release());

    model.initSystem();

    problem.setModelCopy(model);

    TimeInfo timeInfo;
    updateTimeInfo("kinematics",
            kinematics.getIndependentColumn().front(),
            kinematics.getIndependentColumn().back(),
            timeInfo);
    // const double spaceForFiniteDiff = 1e-3;
    problem.setTimeBounds(timeInfo.initial, timeInfo.final);

    // TODO: Allow users to specify costs flexibly.
    problem.addCost<MocoControlCost>("effort");

    auto& solver = moco.initCasADiSolver();
    solver.set_dynamics_mode("implicit");
    solver.set_optim_convergence_tolerance(1e-3);
    solver.set_optim_constraint_tolerance(1e-3);
    // The sparsity detection works fine with DeGrooteFregly2016Muscle.
    solver.set_optim_sparsity_detection("random");
    // solver.set_optim_hessian_approximation("exact");
    // Forward is 3x faster than central.
    solver.set_optim_finite_difference_scheme("forward");
    solver.set_transcription_scheme("trapezoidal");

    solver.set_num_mesh_points(timeInfo.numMeshPoints);
    MocoInverseSolution solution;
    solution.setMocoSolution(moco.solve().unseal());
    return solution;
}
