#ifndef OPENSIM_MOCOINVERSE_H
#define OPENSIM_MOCOINVERSE_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoInverse.h                                                     *
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

#include "MocoStudy.h"
#include "MocoTool.h"
#include "MocoTrajectory.h"
#include "OpenSim/Simulation/TableProcessor.h"
#include "OpenSim/Simulation/PositionMotion.h"
#include "osimMocoDLL.h"

#include <OpenSim/Simulation/Model/Model.h>

namespace OpenSim {

class MocoInverse;

/** This class holds the solution from MocoInverse. */
class MocoInverseSolution {
public:
    const MocoSolution& getMocoSolution() const { return m_mocoSolution; }
    const TimeSeriesTable& getOutputs() const { return m_outputs; }
private:
    void setMocoSolution(MocoSolution mocoSolution) {
        m_mocoSolution = std::move(mocoSolution);
    }
    void setOutputs(TimeSeriesTable outputs) {
        m_outputs = std::move(outputs);
    }
    MocoSolution m_mocoSolution;
    TimeSeriesTable m_outputs;
    friend class MocoInverse;
};

/** This tool solves problems in which the kinematics are prescribed and you
seek the actuator (e.g., muscle) behavior that may have given rise to the
provided kinematics. The term "inverse" describes methods that estimate
quantities from an observation; on the other hand, "forward" methods attempt
to predict (unobserved) behavior. In this case, "inverse" refers to the
multibody systems. This class can still be used to simulate muscles in a
"forward" or predictive sense.

The kinematics file must provide values for all coordinates (even those
labeled as dependent in a CoordinateCouplerConstraint); missing coordinates
are set to NaN.

The provided trajectory is altered to satisfy any enabled kinematic
constraints in the model.

# Cost
By default, MocoInverse minimizes the sum of squared controls and
constrains initial activation to be equal to initial excitation (to avoid
initial activation spikes). To customize the cost, invoke initialize(), add
costs manually, and solve the problem using the solver directly. Note,
however, that kinematic states are not included in the solution if you use
the solver directly.

# Default solver settings
- solver: MocoCasADiSolver
- multibody_dynamics_mode: implicit
- interpolate_control_midpoints: false
- minimize_implicit_auxiliary_derivatives: true
- implicit_auxiliary_derivatives_weight: 0.01
- optim_convergence_tolerance: 1e-3
- optim_constraint_tolerance: 1e-3
- optim_sparsity_detection: random
- optim_finite_difference_scheme: forward

If you would like to use settings other than these defaults, see "Customizing
a problem" below.

MocoInverse minimizes the sum of squared controls and, optionally, the sum
of squared activations. As MocoInverse becomes more mature and general, the
costs will become more flexible.

# Mesh interval
A smaller mesh interval increases the convergence time, but is necessary
for fast motions or problems with stiff differential equations (e.g.,
stiff tendons).
For gait, consider using a mesh interval between 0.01 and 0.05 seconds.
Try solving your problem with decreasing mesh intervals and choose a mesh
interval at which the solution stops changing noticeably.

# Basic example

This example shows how to use MocoInverse in C++:

@code
MocoInverse inverse;
inverse.setName("prescribed_motion");
inverse.setModel(ModelProcessor("model_file.osim") |
                 ModOpAddExternalLoads("external_loads.xml") |
                 ModOpAddReserves());
inverse.setKinematics(TableProcessor("states_reference_file.sto"));
inverse.set_mesh_interval(0.02);
MocoInverseSolution solution = inverse.solve();
solution.getMocoSolution().write("MocoInverse_solution.sto");
@endcode

Customizing the tool
--------------------
The example below shows how you can customize the MocoInverse tool by obtaining
the underlying MocoStudy. You can change the "Default solver settings" above
and add additional goals.

@code
MocoInverse inverse;
inverse.setName("prescribed_motion");
inverse.setModel(ModelProcessor("model_file.osim") |
                 ModOpAddExternalLoads("external_loads.xml") |
                 ModOpAddReserves(1));
inverse.setKinematics(TableProcessor("states_reference_file.sto"));
inverse.set_mesh_interval(0.02);
MocoStudy study = inverse.initialize();
MocoProblem& problem = study.updProblem();
auto* emg_tracking = problem.addGoal<MocoControlTrackingGoal>("emg_tracking");
emg_tracking->setReference(TimeSeriesTable("electromyography.sto"));
auto& solver = study.updSolver<MocoCasADiSolver>();
solver.set_optim_convergence_tolerance(1e-4);
MocoSolution solution = study.solve();
solution.write("MocoInverse_solution.sto");
@endcode

Do NOT change the multibody_dynamics_mode solver setting, as setting this to
"implicit" is vital to how MocoInverse works.

# Path constraints
If adding a MocoPathConstraint to a custom MocoInverse problem, you may want
to enable the solver setting 'enforce_path_constraint_midpoints':

@code
solver.set_enforce_path_constraint_midpoints(true);
@endcode

This is because we do not enforce MocoPathConstraints at mesh interval
midpoints by default with Hermite-Simpson collocation, and the property
'interpolate_control_midpoints' is set to false with MocoInverse to ensure
the problem does not become over-constrained.

For example, if using a MocoControlBoundConstraint with MocoInverse,
the constraint will be ignored at mesh interval midpoints if
'enforce_path_constraint_midpoints' is set to false.
 */
class OSIMMOCO_API MocoInverse : public MocoTool {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoInverse, MocoTool);

public:

    OpenSim_DECLARE_PROPERTY(kinematics, TableProcessor,
            "Generalized coordinate values to prescribe.");

    OpenSim_DECLARE_PROPERTY(kinematics_allow_extra_columns, bool,
            "Allow the kinematics file to contain columns that do not name "
            "states in the model. "
            "This is false by default to help you avoid accidents.");

    OpenSim_DECLARE_PROPERTY(minimize_sum_squared_activations, bool,
            "Minimize the sum of squared activations. Default: false.");

    OpenSim_DECLARE_OPTIONAL_PROPERTY(max_iterations, int,
            "Maximum number of solver iterations (default: solver default).");

    OpenSim_DECLARE_PROPERTY(convergence_tolerance, double,
            "The convergence tolerance (default: 1e-3).");

    OpenSim_DECLARE_PROPERTY(constraint_tolerance, double,
            "The constraint tolerance (default: 1e-3).");

    OpenSim_DECLARE_LIST_PROPERTY(output_paths, std::string,
            "Outputs to compute after solving the problem."
            " Entries can be regular expressions (e.g., '.*activation').");

    OpenSim_DECLARE_PROPERTY(reserves_weight, double,
            "The weight applied to the controls whose name includes "
            "'/reserve_'. This can be used with "
            "the model operator ModOpAddReserves, which names each appended "
            "actuator in this format. Default weight: 1.");

    MocoInverse() { constructProperties(); }

    void setKinematics(TableProcessor kinematics) {
        set_kinematics(std::move(kinematics));
    }

    MocoStudy initialize() const;
    /// Solve the problem returned by initialize() and compute the outputs
    /// listed in output_paths.
    MocoInverseSolution solve() const;

private:
    void constructProperties();
    std::pair<MocoStudy, TimeSeriesTable> initializeInternal() const;
};

} // namespace OpenSim

#endif // OPENSIM_MOCOINVERSE_H
