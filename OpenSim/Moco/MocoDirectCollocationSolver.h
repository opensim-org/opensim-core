#ifndef OPENSIM_MOCODIRECTCOLLOCATIONSOLVER_H
#define OPENSIM_MOCODIRECTCOLLOCATIONSOLVER_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoDirectCollocationSolver.h                                     *
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

/** This is a base class for solvers that use direct collocation to convert
an optimal control problem into a generic nonlinear programming problem.
The best resource for learning about direct collocation is the Betts
textbook:

Betts, John T. Practical methods for optimal control and estimation using
nonlinear programming. Vol. 19. Siam, 2010.

MocoDirectCollocationSolver
===========================
Transcription scheme
--------------------
The `transcription_scheme` setting allows you to choose between
'trapezoidal' and 'hermite-simpson' transcription schemes. The 'trapezoidal'
option replaces the dynamics differential constraints with finite
differences based on trapezoidal rule integration. The 'hermite-simpson'
option uses a Hermite interpolant and Simpson integration to construct the
finite differences. The 'hermite-simpson' option uses the separated
Hermite-Simpson transcription approach, which allows control values at mesh
interval midpoints to be free variables (see Betts textbook for more
details). The setting `interpolate_control_midpoints` constrains control
midpoint variables to be linearly interpolated from the mesh interval
endpoint values (default and recommended setting). If solving problems
including model kinematic constraints, the 'hermite-simpson' option is
required (see Kinematic constraints section below).

Path constraints on controls with Hermite-Simpson transcription
---------------------------------------------------------------
For Hermite-Simpson transcription, the direct collocation solvers enforce
the path constraints (e.g., MocoPathConstraint) at only the mesh interval
endpoints (not midpoints), but control signal variables exist at both mesh
interval endpoints and midpoints. Keep this in mind when using path
constraints on controls (e.g., MocoControlBoundConstraint). If
`interpolate_control_midpoints` is false, the values of a control at
midpoints may differ greatly from the values at mesh interval endpoints.

Multibody dynamics mode
-----------------------
The `multibody_dynamics_mode` setting allows you to choose between
expressing multibody dynamics as explicit differential equations (e.g., \f$
\dot{y} = f(y) \f$) or implicit differential equations (e.g., \f$ 0 = f(y,
\dot{y}) \f$, or inverse dynamics). Whether auxiliary dynamics (e.g.,
muscle fiber and activation dynamics) are implicit or explicit depends on
the model component implementing those dynamics.

Kinematic constraints
---------------------
All holonomic kinematic constraints included as OpenSim model constraints are
supported. Both the 'trapezoidal' and 'hermite-simpson' transcription schemes
support kinematic constraints, but the 'hermite-transcription' scheme handles
kinematic constraints much more robustly; in practice, the 'trapezoidal' scheme
is not used for models with kinematic constraints. Kinematic constraints are
automatically detected if present in the model and are converted to path
constraints in the optimal control problem based on the method presented in Posa
et al. 2016, 'Optimization and stabilization of trajectories for constrained
dynamical systems'; see @ref implkincon. The `minimize_lagrange_multipliers` and
`lagrange_multiplier_weight` settings allow you to enable and set the weight for
the minimization of all Lagrange multipliers associated with kinematic
constraints in the problem. The `velocity_correction_bounds` setting allows you
to set the bounds on the velocity correction variables that project state
variables onto the constraint manifold when necessary to properly enforce defect
constraints (see Posa et al. 2016 for details). */
class OSIMMOCO_API MocoDirectCollocationSolver : public MocoSolver {
    OpenSim_DECLARE_ABSTRACT_OBJECT(MocoDirectCollocationSolver, MocoSolver);

public:
    OpenSim_DECLARE_PROPERTY(num_mesh_intervals, int,
            "The number of uniformly-sized mesh intervals for the problem "
            "(default: 100). If a non-uniform mesh exists, the non-uniform "
            "mesh is used instead.");

    OpenSim_DECLARE_PROPERTY(verbosity, int,
            "0 for silent. 1 for only Moco's own output. "
            "2 for output from CasADi and the underlying solver (default: 2).");
    OpenSim_DECLARE_PROPERTY(transcription_scheme, std::string,
            "'trapezoidal' for trapezoidal transcription, or 'hermite-simpson' "
            "(default) for separated Hermite-Simpson transcription.");
    OpenSim_DECLARE_PROPERTY(interpolate_control_midpoints, bool,
            "If the transcription scheme is set to 'hermite-simpson', then "
            "enable this property to constrain the control values at mesh "
            "interval midpoints to be linearly interpolated from the control "
            "values at the mesh interval endpoints. Default: true.");
    OpenSim_DECLARE_PROPERTY(multibody_dynamics_mode, std::string,
            "Multibody dynamics are expressed as 'explicit' (default) or "
            "'implicit' differential equations.");
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
            "'true' (default) or 'false', whether or not derivatives of "
            "kinematic constraints are enforced as path constraints in the "
            "optimal control problem.");
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
    OpenSim_DECLARE_PROPERTY(implicit_multibody_acceleration_bounds, MocoBounds,
            "Bounds on acceleration variables in implicit dynamics mode. "
            "Default: [-1000, 1000]");
    OpenSim_DECLARE_PROPERTY(implicit_auxiliary_derivative_bounds, MocoBounds,
            "Bounds on derivative variables for components with auxiliary "
            "dynamics in implicit form. Default: [-1000, 1000]");

    MocoDirectCollocationSolver() { constructProperties(); }

    /** %Set the mesh to a user-defined list of mesh points to sample. This
     * takes precedence over the uniform mesh that would be specified with
     * num_mesh_intervals. The user-defined mesh must start with 0, be strictly
     * increasing (no duplicate entries), and end with 1. */
    void setMesh(const std::vector<double>& mesh);

protected:
    OpenSim_DECLARE_PROPERTY(guess_file, std::string,
            "A MocoTrajectory file storing an initial guess.");
    OpenSim_DECLARE_LIST_PROPERTY(mesh, double,
            "Usually non-uniform, user-defined list of mesh points to sample. "
            "Takes precedence over uniform mesh with num_mesh_intervals.");
    void constructProperties();
};

} // namespace OpenSim

#endif // OPENSIM_MOCODIRECTCOLLOCATIONSOLVER_H
