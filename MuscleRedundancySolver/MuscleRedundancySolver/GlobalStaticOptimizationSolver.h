#ifndef TOMU_GLOBALSTATICOPTIMIZATIONSOLVER_H
#define TOMU_GLOBALSTATICOPTIMIZATIONSOLVER_H

#include "InverseMuscleSolver.h"

#include <OpenSim/OpenSim.h>

namespace OpenSim {

// TODO document
// TODO example usage.
class GlobalStaticOptimizationSolver : public InverseMuscleSolver {
    OpenSim_DECLARE_CONCRETE_OBJECT(GlobalStaticOptimizationSolver,
            InverseMuscleSolver);
public:

    // TODO rename to Iterate?
    struct Solution {
        /// The activation trajectories for all enabled (appliesForce) muscles.
        /// This will be empty if there are no enabled muscles.
        TimeSeriesTable activation;
        /// The control for enabled (appliesForce) CoordinateActuators, etc.
        /// This will be empty if there are no CoordinateActuators, etc.
        /// enabled.
        TimeSeriesTable other_controls;
        // TODO could have separate functions to compute these length/vel tables
        // given a Solution.
        /// This is not one of the variables in the optimization problem,
        /// but may be interesting nonetheless.
        TimeSeriesTable norm_fiber_length;
        /// This is not one of the variables in the optimization problem,
        /// but may be interesting nonetheless.
        TimeSeriesTable norm_fiber_velocity;
        /// This is not one of the variables in the optimization problem,
        /// but may be interesting nonetheless. In units of Newtons.
        TimeSeriesTable tendon_force;
        void write(const std::string& prefix) const;
    };

    GlobalStaticOptimizationSolver() = default;

    /// Load a solver from an XML setup file.
    ///
    /// Note: Use print() to save solver settings to an XML file that can
    /// be subsequently loaded via this constructor.
    ///
    /// If the model or kinematics files are provided, they are not
    /// loaded until you call solve(). If these files are not provided, you
    /// must call the setModel() and/or setKinematicsData() functions before
    /// calling solve().
    explicit GlobalStaticOptimizationSolver(const std::string& setupFilePath);

    /// Solve for muscle activity. You must have provide a model and
    /// kinematics data before calling this function.
    /// @returns A struct containing muscle activation, fiber
    ///     length, fiber velocity, tendon force, and other control signals.
    Solution solve();

};

} // namespace OpenSim

#endif // TOMU_GLOBALSTATICOPTIMIZATIONSOLVER_H
