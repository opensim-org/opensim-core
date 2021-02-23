#ifndef OPENSIM_CASOCITERATE_H
#define OPENSIM_CASOCITERATE_H
/* -------------------------------------------------------------------------- *
 * OpenSim: CasOCIterate.h                                                    *
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

#include <casadi/casadi.hpp>

namespace CasOC {

/// This enum describes the different types of optimization variables, and
/// are the keys for the Variables map.
enum Var {
    initial_time,
    final_time,
    /// Differential variables.
    states,
    /// Algebraic variables.
    controls,
    /// Used for kinematic constraints.
    multipliers,
    /// Used for certain methods of solving kinematic constraints.
    slacks,
    /// Used in implicit dynamics mode.
    derivatives, // TODO: Rename to accelerations?
    /// Constant in time.
    parameters,
    /// For internal use (never actually a key for Variables).
    multibody_states = 100
};

template <typename T>
using Variables = std::unordered_map<Var, T, std::hash<int>>;

/// Numeric variables for initial guesses and solutions.
using VariablesDM = Variables<casadi::DM>;
/// Symbolic variables, used to define the problem.
using VariablesMX = Variables<casadi::MX>;

/// This struct is used to obtain initial guesses.
struct Iterate {
    VariablesDM variables;
    casadi::DM times;
    std::vector<std::string> state_names;
    std::vector<std::string> control_names;
    std::vector<std::string> multiplier_names;
    std::vector<std::string> slack_names;
    std::vector<std::string> derivative_names;
    std::vector<std::string> parameter_names;
    int iteration = -1;
    /// Return a new iterate in which the data is resampled at the times in
    /// newTimes.
    Iterate resample(const casadi::DM& newTimes) const;
};

/// This struct is used to return a solution to a problem. Use `stats`
/// to check if the problem converged.
using ObjectiveBreakdown = std::vector<std::pair<std::string, double>>;
struct Solution : public Iterate {
    casadi::Dict stats;
    double objective;
    ObjectiveBreakdown objective_breakdown;
};

} // namespace CasOC

#endif // OPENSIM_CASOCITERATE_H
