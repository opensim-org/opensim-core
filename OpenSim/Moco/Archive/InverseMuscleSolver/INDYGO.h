#ifndef OPENSIM_INDYGO_H
#define OPENSIM_INDYGO_H
/* -------------------------------------------------------------------------- *
 * OpenSim: INDYGO.h                                                          *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
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

#include "InverseMuscleSolver.h"

namespace OpenSim {

// TODO if this solves for the activity of any actuator, then it should not
// be called "muscle" redundancy solver.
/// TODO describe filtering.

// TODO create the option to directly specify inverse dynamics generalized
// forces, so that users can process them however they desire.

// TODO replace optimal force with something easier to interpret: controls
// between 0 and 1, and weights in the objective function.
// TODO method performs better with longer activation/deactivation time
// constants. TODO get these constants from the muscle model rather than using
// hard-coded values.

/**
Mesh point frequency
--------------------
This is a property in the base class, InverseMuscleSolver.
For gait, you should use between 100-300 mesh points per second. Having too
few mesh points may result in spiky solutions, especially for fiber
velocity. A higher mesh point frequency often results in a smoother and more
accurate solution, but may cause the problem to take longer to solve. Faster
motions (with faster dynamics) require a higher mesh point frequency.
The "tendon_force" fiber dynamics mode may require a higher mesh point
frequency than the "fiber_length" mode to obtain a comparable level of
smoothness (TODO look into this more).

Fiber dynamics mode
-------------------
This mode was shown to be the fastest of the 4 formulations in [1].
Instead of using fiber length as a state variable, the (normalized
tendon) force is used instead. We introduce a control variable,
tendon_force_rate_control, that sets the derivative of the tendon force.
Fiber-tendon equilibrium is still enforced.
The tendon_force_rate_control variable has bounds [-50, 50], and the
norm_tendon_force state variable has bounds [0, 5].

Activation dynamics mode
------------------------
TODO not implemented yet.
There are two choices for the formulation of activation dynamics:
"explicit" and "implicit". The implicit mode may be up to 2 times faster
than the explicit mode, and you should try the implict mode first.
The explicit mode is based on [1], while the implicit mode is based on
aspects of [2] (see also https://simtk.org/projects/optcntrlmuscle
version 1.1). In the explicit model, excitation is a control variable
and activation dynamics is one of the differential equations in the optimal
control problem. The implicit mode is based on the realization that one
doesn't actually need to simulate activation dynamics within the optimal
control problem; rather, excitation can be computed post-hoc so long as
the optimal control problem obeys the upper and lower bounds on the rate of
activation. Therefore, the implicit mode does not have a variable for
excitation, and computes excitation post-hoc by inverting the activation
dynamics differential equation.

The model for activation dynamics is different in these two modes. The
implicit mode uses the activation dynamics model from [3]:

\f{eqnarray*}{
\frac{da}{dt} = \begin{cases}
(u - a) \left( \frac{u}{\tau_a} + \frac{1 - u}{\tau_d} \right) & u \geq a \\\\
\frac{u - a}{\tau_d}                                           & u < a
\end{cases}
\f}
where \f$ u \f$ is excitation, \f$ a \f$ is activation, and \f$ \tau_a \f$
and \f$ \tau_d \f$ are the activation and deactivation time constants.
We add a control variable $v$ that is the rate of change of activation and
supply the following (trivial) dynamics in the optimal control problem:
\f[
    \frac{da}{dt} = v
\f]
The following inequality path constraints are used to ensure that the
activation doesn't change more quickly than activation dynamics would
allow (obtained by setting \f$ u = 0 \f$ and \f$ u = 1 \f$):

\f{eqnarray*}{
    v + \frac{a}{\tau_d} &\geq 0 \\\\
    v + \frac{a}{\tau_a} &\leq \frac{1}{\tau_a}
\f}

In the explicit mode, the cost is the sum of squared *excitations*. In
the implicit mode, the cost is the sum of squared *activations*. In both
cases, the cost for non-muscle actuators is their control signal.

TODO move this information to a user's guide.
Notes
-----
1. If the activation dynamics mode is explicit, the cost functional includes
   the sum of squared excitations *and* the sum of squared activations
   (as well as the sum of squared "other controls").
   Minimizing excitations prevents a spiky solution, and minimizing
   activations prevents the initial activation from being unreasonably
   large (since initial activation would otherwise be "free" in the
   optimization).
2. The muscle's min_control and max_control properties are ignored (non-zero
   min activation worsens the numerical properties of the problem).

[1] De Groote, Friedl, et al. "Evaluation of direct collocation optimal
control problem formulations for solving the muscle redundancy problem."
Annals of biomedical engineering 44.10 (2016): 2922-2936.

[2] De Groote, Friedl, et al. "A physiology based inverse dynamic analysis of
human gait: potential and perspectives." Computer methods in biomechanics and
biomedical engineering 12.5 (2009): 563-574.

[3] Raasch, Christine C., et al. "Muscle coordination of maximum-speed
pedaling." Journal of biomechanics 30.6 (1997): 595-602. */
class OSIMMOCO_API INDYGO : public InverseMuscleSolver {
    OpenSim_DECLARE_CONCRETE_OBJECT(INDYGO, InverseMuscleSolver);
public:

    OpenSim_DECLARE_PROPERTY(initial_guess, std::string,
        "How to compute the initial guess for the optimal control problem. "
        "'static_optimization': Solve a static optimization problem to obtain "
                "a guess. "
        "'bounds': The guess is the midpoint of the variables' constraints. "
        "Default: 'static_optimization'. "
        "Only change this if static optimization is causing issues; static "
        " optimization can help solve the problem up 10 times faster.");

    // TODO consider allowing setting initial fiber velocity instead, though
    // it is not as good at avoiding poor initial activations, etc.
    OpenSim_DECLARE_PROPERTY(zero_initial_activation, bool,
        "Should the initial activation of all muscles be constrained to 0? "
        "You should only set this true if you have reason to believe that "
        "initial activation should be 0 and that doesn't occur when "
        "this is set false."
        "Default: false");

    // TODO these options would move into the muscle classes.
    // TODO change the default to tendon_force.
    OpenSim_DECLARE_PROPERTY(fiber_dynamics_mode, std::string,
        "The formulation of fiber dynamics. "
        "'fiber_length': use normalized fiber length as a state variable. "
        "'tendon_force': use normalized tendon force as a state variable. "
        "Using tendon force is usually much faster. "
        "Default: 'fiber_length'.");
    // TODO change the default to implicit.
    OpenSim_DECLARE_PROPERTY(activation_dynamics_mode, std::string,
        "The formulation of activation dynamics. "
        "'explicit': excitation is a control variable and activation dynamics "
                "are simulated in the optimal control problem. "
        "'implicit': excitation is computed post-hoc. "
        "The implicit mode may be up to 2 times faster. "
        "Default: 'explicit'.");

    struct OSIMMOCO_API Solution {
        /// The excitation trajectories for all enabled (appliesForce) muscles.
        /// This will be empty if there are no enabled muscles.
        TimeSeriesTable excitation;
        /// The activation trajectories for all enabled (appliesForce) muscles.
        /// This will be empty if there are no enabled muscles.
        TimeSeriesTable activation;
        /// The fiber length trajectories for all enabled (appliesForce)
        /// muscles, normalized by optimal fiber length.
        /// This is a state variable if fiber_dynamics_mode=="fiber_length".
        /// This will be empty if there are no enabled muscles.
        TimeSeriesTable norm_fiber_length;
        /// The fiber velocity trajectories for all enabled (appliesForce)
        /// muscles, normalized by max contraction velocity (e.g., within
        /// [-1, 1]). This is a control variable if
        /// fiber_dynamics_mode=="fiber_length".
        /// This will be empty if there are no enabled muscles.
        TimeSeriesTable norm_fiber_velocity;
        /// The control for enabled (appliesForce) CoordinateActuators, etc.
        /// This will be empty if there are no CoordinateActuators, etc.
        /// enabled.
        TimeSeriesTable other_controls;
        /// The tendon force trajectories for all enabled (appliesForce)
        /// muscles. This is *not* a variable in INDYGO; rather, this is
        /// provided to support analysis of results.
        /// This will be empty if there are no enabled muscles.
        TimeSeriesTable tendon_force;
        /// The normalized tendon force trajectories for all enabled
        /// (appliesForce) muscles. This is a state variable if
        /// fiber_dynamics_mode=="tendon_force".
        /// This will be empty if there are no enabled muscles.
        TimeSeriesTable norm_tendon_force;
        // The control variable used when fiber_dynamics_mode=="tendon_force".
        // This is a scaled version of the rate of change of tendon force.
        // This is not used when fiber_dynamics_mode=="fiber_length".
        TimeSeriesTable tendon_force_rate_control;
        /// Write the solution to a series of OpenSim Storage files (one
        /// for each table) whose paths begin with `prefix`. Empty tables are
        /// not written.
        void write(const std::string& prefix) const;
    };

    INDYGO();

    /// Load a solver from an XML setup file.
    ///
    /// Note: Use print() to save solver settings to an XML file that can
    /// be subsequently loaded via this constructor.
    ///
    /// If the model or kinematics files are provided, they are not
    /// loaded until you call solve(). If these files are not provided, you
    /// must call the setModel() and/or setKinematicsData() functions before
    /// calling solve().
    explicit INDYGO(const std::string& setupFilePath);

    /// Solve for muscle activity. You must have provide a model and
    /// kinematics data before calling this function. If the property
    /// `write_solution` is not 'false', then the solution is also written as
    /// files to the disk.
    /// @returns A struct containing muscle excitation, activation, fiber
    ///     length, fiber velocity, and other control signals.
    Solution solve() const;

private:
    void constructProperties();

};

} // namespace OpenSim

#endif // OPENSIM_INDYGO_H
