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
#include "MocoProblem.h"
#include "MocoTool.h"
#include "MocoUtilities.h"

#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Tools/InverseDynamicsTool.h>

using namespace OpenSim;

void MocoInverse::constructProperties() {

    constructProperty_initial_time();
    constructProperty_final_time();
    constructProperty_mesh_point_frequency(50);
    constructProperty_create_reserve_actuators(-1);
    constructProperty_external_loads_file("");
}

MocoInverseSolution MocoInverse::solve() const {

    // TODO: Create an initial guess using GSO (no dynamics).

    Model model(m_model);

    MocoTool moco;
    auto& problem = moco.updProblem();

    model.finalizeFromProperties();

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

    auto kinematicsRaw = STOFileAdapter::read(m_kinematicsFileName);
    if (kinematicsRaw.hasTableMetaDataKey("inDegrees") &&
            kinematicsRaw.getTableMetaDataAsString("inDegrees") == "yes") {
        model.getSimbodyEngine().convertDegreesToRadians(kinematicsRaw);
    }
    auto kinematics = filterLowpass(kinematicsRaw, 6, true);

    auto posmot = PositionMotion::createFromTable(model, kinematics, true);
    posmot->setName("position_motion");
    model.addComponent(posmot.release());

    model.initSystem();
    if (get_create_reserve_actuators() != -1) {
        createReserveActuators(model, get_create_reserve_actuators());
    }

    problem.setModelCopy(model);

    // const double spaceForFiniteDiff = 1e-3;
    // problem.setTimeBounds(kinematicsRaw.getIndependentColumn().front() +
    //                               spaceForFiniteDiff,
    //         kinematicsRaw.getIndependentColumn().back() -
    //                 spaceForFiniteDiff);
    const auto timeInfo =
            calcInitialAndFinalTimes(kinematicsRaw.getIndependentColumn(), {},
                    get_mesh_point_frequency());
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
        const int& meshPointFrequency) const {

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
    out.numMeshPoints = (int)std::ceil(
            (out.finalTime - out.initialTime) * meshPointFrequency);
    return out;
}
