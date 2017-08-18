#ifndef MUSCOLLO_MUSCLEREDUNDANCYSOLVER_H
#define MUSCOLLO_MUSCLEREDUNDANCYSOLVER_H

#include "InverseMuscleSolver.h"

#include <OpenSim/OpenSim.h>

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
class MuscleRedundancySolver : public InverseMuscleSolver {
    OpenSim_DECLARE_CONCRETE_OBJECT(MuscleRedundancySolver,
            InverseMuscleSolver);
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
        "This may be useful because the optimizer will often otherwise set "
        "initial activation to an unreasonably large value (there is no cost "
        "for this). "
        "Default: false");
    // TODO not true anymore: If a muscle's min_control is greater than 0,"
    //      " then min_control is used instead. "

    struct Solution {
        /// The excitation trajectories for all enabled (appliesForce) muscles.
        /// This will be empty if there are no enabled muscles.
        TimeSeriesTable excitation;
        /// The activation trajectories for all enabled (appliesForce) muscles.
        /// This will be empty if there are no enabled muscles.
        TimeSeriesTable activation;
        /// The fiber length trajectories for all enabled (appliesForce)
        /// muscles, normalized by optimal fiber length.
        /// This will be empty if there are no enabled muscles.
        TimeSeriesTable norm_fiber_length;
        /// The fiber length trajectories for all enabled (appliesForce)
        /// muscles, normalized by max contraction velocity (e.g., within
        /// [-1, 1]). This will be empty if there are no enabled muscles.
        TimeSeriesTable norm_fiber_velocity;
        /// The control for enabled (appliesForce) CoordinateActuators, etc.
        /// This will be empty if there are no CoordinateActuators, etc.
        /// enabled.
        TimeSeriesTable other_controls;
        /// The tendon force trajectories for all enabled (appliesForce)
        /// muscles. This is *not* a variable in the MuscleRedundancySolver;
        /// rather, this is provided to support analysis of results.
        /// This will be empty if there are no enabled muscles.
        TimeSeriesTable tendon_force;
        /// Write the solution to a series of OpenSim Storage files (one
        /// for each table) whose paths begin with `prefix`. Empty tables are
        /// not written.
        void write(const std::string& prefix) const;
    };

    MuscleRedundancySolver();

    /// Load a solver from an XML setup file.
    ///
    /// Note: Use print() to save solver settings to an XML file that can
    /// be subsequently loaded via this constructor.
    ///
    /// If the model or kinematics files are provided, they are not
    /// loaded until you call solve(). If these files are not provided, you
    /// must call the setModel() and/or setKinematicsData() functions before
    /// calling solve().
    explicit MuscleRedundancySolver(const std::string& setupFilePath);

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

#endif // MUSCOLLO_MUSCLEREDUNDANCYSOLVER_H
