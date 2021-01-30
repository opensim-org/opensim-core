#ifndef OPENSIM_GLOBALSTATICOPTIMIZATION_H
#define OPENSIM_GLOBALSTATICOPTIMIZATION_H
/* -------------------------------------------------------------------------- *
 * OpenSim: GlobalStaticOptimization.h                                        *
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

#include <OpenSim/Common/osimCommon.h>

namespace OpenSim {

// TODO document
// TODO example usage.
class OSIMMOCO_API GlobalStaticOptimization : public InverseMuscleSolver {
    OpenSim_DECLARE_CONCRETE_OBJECT(GlobalStaticOptimization,
            InverseMuscleSolver);
public:

    // TODO rename to Iterate?
    struct OSIMMOCO_API Solution {
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
        /// The force in each muscle's tendon, in units of Newtons.
        /// This is not one of the variables in the optimization problem,
        /// but may be interesting nonetheless.
        TimeSeriesTable tendon_force;
        /// The force in each muscle's tendon, normalized by maximum isometric
        /// force. This is not one of the variables in the optimization problem,
        /// but may be interesting nonetheless.
        TimeSeriesTable norm_tendon_force;
        void write(const std::string& prefix) const;
    };

    GlobalStaticOptimization() = default;

    /// Load a solver from an XML setup file.
    ///
    /// Note: Use print() to save solver settings to an XML file that can
    /// be subsequently loaded via this constructor.
    ///
    /// If the model or kinematics files are provided, they are not
    /// loaded until you call solve(). If these files are not provided, you
    /// must call the setModel() and/or setKinematicsData() functions before
    /// calling solve().
    explicit GlobalStaticOptimization(const std::string& setupFilePath);

    /// Solve for muscle activity. You must have provide a model and
    /// kinematics data before calling this function. If the property
    /// `write_solution` is not 'false', then the solution is also written as
    /// files to the disk.
    /// @returns A struct containing muscle activation, fiber
    ///     length, fiber velocity, tendon force, and other control signals.
    Solution solve() const;

};

} // namespace OpenSim

#endif // OPENSIM_GLOBALSTATICOPTIMIZATION_H
