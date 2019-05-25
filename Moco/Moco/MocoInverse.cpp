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
    constructProperty_initial_time();
    constructProperty_final_time();
    constructProperty_mesh_interval(0.02);
    constructProperty_model(ModelProcessor());
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

    model.initSystem();

    TimeSeriesTable kinematics = get_kinematics().process(setupDir,
            &model);

    // allowMissingColumns = true: we only need kinematics.
    // allowExtraColumns = false: user might have made an error.
    // assemble = true: we must obey the kinematic constraints.
    auto statesTraj = StatesTrajectory::createFromStatesStorage(
            model, convertTableToStorage(kinematics), true, false, true);

    auto posmot = PositionMotion::createFromStatesTrajectory(
            model, statesTraj);
    posmot->setName("position_motion");
    model.addComponent(posmot.release());

    model.initSystem();

    problem.setModelCopy(model);

    const auto timeInfo = calcInitialAndFinalTimes(
            kinematics.getIndependentColumn(), {}, get_mesh_interval());
    // const double spaceForFiniteDiff = 1e-3;
    problem.setTimeBounds(timeInfo.initialTime, timeInfo.finalTime);

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

    solver.set_num_mesh_points(timeInfo.numMeshPoints);
    MocoInverseSolution solution;
    solution.setMocoSolution(moco.solve().unseal());
    // TimeSeriesTable normFiberLengths =
    //         moco.analyze(solution, {".*normalized_fiber_length"});
    // STOFileAdapter::write(normFiberLengths,
    //         "sandboxWalkingStatelessMuscles_norm_fiber_length.sto");
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
