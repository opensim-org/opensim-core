#ifndef MOCO_MOCODIRECTCOLLOCATIONSOLVER_H
#define MOCO_MOCODIRECTCOLLOCATIONSOLVER_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoDirectCollocationSolver.h                                *
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

#include "MocoSolver.h"

#include <OpenSim/Common/Object.h>

namespace OpenSim {

/// This is a base class for solvers that use direct collocation to convert
/// an optimal control problem into a generic nonlinear programming problem.
/// The best resource for learning about direct collocation is the Betts
/// textbook.
///
/// Dynamics mode
/// -------------
/// The `dynamics_mode` setting allows you to choose between expressing
/// multibody dynamics as explicit differential equations (e.g., \f$ \dot{y} =
/// f(y) \f$) or implicit differential equations (e.g., \f$ 0 = f(y, \dot{y})
/// \f$, or inverse dynamics). Currently, auxiliary dynamics (e.g., muscle fiber
/// and activation dynamics) are always explicit. Betts, John T. Practical
/// methods for optimal control and estimation using nonlinear programming.
/// Vol. 19. Siam, 2010.
class MocoDirectCollocationSolver : public MocoSolver {
    OpenSim_DECLARE_ABSTRACT_OBJECT(MocoDirectCollocationSolver, MocoSolver);

public:
    OpenSim_DECLARE_PROPERTY(num_mesh_points, int,
            "The number of mesh points for discretizing the problem (default: "
            "100).");
    OpenSim_DECLARE_PROPERTY(verbosity, int,
            "0 for silent. 1 for only Moco's own output. "
            "2 for output from CasADi and the underlying solver (default: 2).");
    OpenSim_DECLARE_PROPERTY(transcription_scheme, std::string,
            "'trapezoidal' (default) for trapezoidal transcription, or "
            "'hermite-simpson' for separated Hermite-Simpson transcription.");
    OpenSim_DECLARE_PROPERTY(dynamics_mode, std::string,
            "Dynamics are expressed as 'explicit' (default) or 'implicit' "
            "differential equations.");
    OpenSim_DECLARE_PROPERTY(optim_solver, std::string,
            "The optimization solver to use (default: ipopt).");
    OpenSim_DECLARE_PROPERTY(optim_max_iterations, int,
            "Maximum number of iterations in the optimization solver "
            "(-1 for solver's default).");
    OpenSim_DECLARE_PROPERTY(optim_convergence_tolerance, double,
            "Tolerance used to determine if the objective is minimized "
            "(-1 for solver's default)");
    OpenSim_DECLARE_PROPERTY(optim_constraint_tolerance, double,
            "Tolerance used to determine if the constraints are satisfied "
            "(-1 for solver's default)");
    OpenSim_DECLARE_PROPERTY(optim_hessian_approximation, std::string,
            "When using IPOPT, 'limited-memory' (default) for quasi-Newton, or "
            "'exact' for full "
            "Newton.");
    OpenSim_DECLARE_PROPERTY(optim_ipopt_print_level, int,
            "IPOPT's verbosity (see IPOPT documentation).");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(enforce_constraint_derivatives, bool,
            "'true' or 'false', whether or not derivatives of kinematic "
            "constraints are enforced as path constraints in the optimal "
            "control problem.");
    OpenSim_DECLARE_PROPERTY(minimize_lagrange_multipliers, bool,
            "If enabled, a term minimizing the weighted, squared sum of "
            "any existing Lagrange multipliers is added to the optimal control "
            "problem. This may be useful for imposing uniqueness in the "
            "Lagrange multipliers when not enforcing model kinematic "
            "constraint derivatives or when the constraint Jacobian is "
            "singular. To set the weight for this term use the "
            "'lagrange_multiplier_weight' property. Default: false");
    OpenSim_DECLARE_PROPERTY(lagrange_multiplier_weight, double,
            "If the 'minimize_lagrange_multipliers' property is enabled, this "
            "defines the weight for the cost term added to the optimal control "
            "problem. Default: 1");
    OpenSim_DECLARE_PROPERTY(velocity_correction_bounds, MocoBounds,
            "For problems where model kinematic constraint derivatives are "
            "enforced, set the bounds on the slack variables performing the "
            "velocity correction to project the model coordinates back onto "
            "the constraint manifold. Default: [-0.1, 0.1]");

    MocoDirectCollocationSolver() { constructProperties(); }

protected:
    OpenSim_DECLARE_PROPERTY(guess_file, std::string,
            "A MocoIterate file storing an initial guess.");

    void constructProperties();
};

} // namespace OpenSim

#endif // MOCO_MOCODIRECTCOLLOCATIONSOLVER_H
