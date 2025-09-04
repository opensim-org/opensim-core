/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoDirectCollocationSolver.cpp                              *
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

#include "MocoDirectCollocationSolver.h"
#include "OpenSim/Common/Logger.h"

using namespace OpenSim;

void MocoDirectCollocationSolver::constructProperties() {
    constructProperty_num_mesh_intervals(100);
    constructProperty_mesh();
    constructProperty_verbosity(2);
    constructProperty_transcription_scheme("hermite-simpson");
    constructProperty_interpolate_control_mesh_interior_points(true);
    constructProperty_enforce_constraint_derivatives(true);
    constructProperty_multibody_dynamics_mode("explicit");
    constructProperty_optim_solver("ipopt");
    constructProperty_optim_max_iterations(-1);
    constructProperty_optim_convergence_tolerance(-1);
    constructProperty_optim_constraint_tolerance(-1);
    constructProperty_optim_hessian_approximation("limited-memory");
    constructProperty_optim_ipopt_print_level(-1);
    constructProperty_guess_file("");
    constructProperty_velocity_correction_bounds({-0.1, 0.1});
    constructProperty_implicit_multibody_acceleration_bounds({-1000, 1000});
    constructProperty_implicit_auxiliary_derivative_bounds({-1000, 1000});
    constructProperty_minimize_lagrange_multipliers(false);
    constructProperty_lagrange_multiplier_weight(1.0);
    constructProperty_kinematic_constraint_method("Posa2016");
}

void MocoDirectCollocationSolver::setMesh(const std::vector<double>& mesh) {
    for (int i = 0; i < (int)mesh.size(); ++i) { set_mesh(i, mesh[i]); }
}

void MocoDirectCollocationSolver::checkConstraintJacobianRank(
        const MocoSolution& mocoSolution) const {
    const auto& model = getProblemRep().getModelBase();
    const auto& matter = model.getMatterSubsystem();
    TimeSeriesTable states = mocoSolution.exportToStatesTable();
    // TODO update when we support multiple phases.
    auto statesTraj =
            StatesTrajectory::createFromStatesTable(model, states);
    SimTK::Matrix G;
    SimTK::FactorQTZ G_qtz;
    bool isJacobianFullRank = true;
    int rank;
    for (const auto& s : statesTraj) {
        // Jacobian is at most velocity-dependent.
        model.realizeVelocity(s);
        matter.calcG(s, G);
        G_qtz.factor<double>(G);
        if (G_qtz.getRank() < G.nrow()) {
            isJacobianFullRank = false;
            rank = G_qtz.getRank();
            break;
        }
    }

    if (!isJacobianFullRank) {
        const std::string dashes(52, '-');
        log_warn(dashes);
        log_warn("Rank-deficient constraint Jacobian detected.");
        log_warn(dashes);
        log_warn("The model constraint Jacobian has {} row(s) but is only "
                 "rank {}. ", G.nrow(), rank);
        log_warn("Try removing redundant constraints from the model or "
                 "enable");
        log_warn("minimization of Lagrange multipliers by utilizing the "
                 "solver ");
        log_warn("properties 'minimize_lagrange_multipliers' and");
        log_warn("'lagrange_multiplier_weight'.");
        log_warn(dashes);
    }
}

void MocoDirectCollocationSolver::checkSlackVariables(
        const MocoSolution& solution) const {
    const auto& slacks = solution.getSlacksTrajectory();
    const auto& slackNames = solution.getSlackNames();
    const SimTK::Real threshold = 1e-3;
    bool largeSlackDetected = false;

    std::vector<std::string> slackWarnings;
    for (int islack = 0; islack < slacks.ncol(); ++islack) {
        if (SimTK::max(SimTK::abs(slacks.col(islack))) > threshold) {
            largeSlackDetected = true;
            std::stringstream ss;
            ss << "Slack variable '" << slackNames[islack] << "' has a maximum "
               << "value of " << SimTK::max(SimTK::abs(slacks.col(islack)))
               << ".";
            slackWarnings.push_back(ss.str());
        }
    }

    if (largeSlackDetected) {
        const std::string dashes(50, '-');
        log_warn(dashes);
        log_warn("Large slack variables detected.");
        log_warn(dashes);
        for (const auto& warning : slackWarnings) {
            log_warn(warning);
        }
        log_warn("");
        log_warn("Slack variables with values larger than ~{} might", threshold);
        log_warn("indicate that the problem is struggling to enforce");
        log_warn("kinematic constraints in the problem. Since slack variables");
        log_warn("interact with defect constraints in the direct collocation ");
        log_warn("formulation, large slack values may affect the quality ");
        log_warn("of the solution. Consider refining the mesh or adjusting ");
        log_warn("the constraint tolerance in the MocoProblem.");
        log_warn(dashes);
    }
} 
