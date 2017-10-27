#ifndef TROPTER_DIRECTCOLLOCATION_H
// ----------------------------------------------------------------------------
// tropter: DirectCollocation.h
// ----------------------------------------------------------------------------
// Copyright (c) 2017 tropter authors
//
// Licensed under the Apache License, Version 2.0 (the "License"); you may
// not use this file except in compliance with the License. You may obtain a
// copy of the License at http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// ----------------------------------------------------------------------------
#define TROPTER_DIRECTCOLLOCATION_H

#include <tropter/common.h>
#include "OptimalControlProblem.h"
#include "OptimalControlIterate.h"
#include <fstream>
#include <memory>

namespace tropter {

class OptimizationSolver;

namespace transcription {
template<typename T>
class Base;
} // namespace transcription

/// @ingroup optimalcontrol
template<typename T>
class DirectCollocationSolver {
public:
    typedef OptimalControlProblem<T> OCProblem;
    DirectCollocationSolver(std::shared_ptr<const OCProblem> ocproblem,
                            const std::string& transcription_method,
                            const std::string& optimization_solver,
                            // TODO remove; put somewhere better.
                            const unsigned& num_mesh_points = 20);
    OptimalControlIterate make_initial_guess_from_bounds() const {
        // We only need this decorator to form an initial guess from the bounds.
        auto decorator = m_transcription->make_decorator();
        return m_transcription->deconstruct_iterate(
                decorator->make_initial_guess_from_bounds());
    }
    /// Get the OptimizationSolver, through which you can query optimizer
    /// settings like maximum number of iterations. This provides only const
    /// access, so it does not let you edit settings of the solver; see the
    /// non-const variant below if you need to change settings.
    const OptimizationSolver& get_optimization_solver() const {
        return *m_optsolver.get();
    }
    /// Get the OptimizationSolver, through which you can set optimizer
    /// settings like maximum number of iterations.
    OptimizationSolver& get_optimization_solver() {
        return *m_optsolver.get();
    }

    /// 0 for silent, 1 for verbose. This setting is copied into the
    /// underlying solver.
    void set_verbosity(int verbosity);
    /// @copydoc set_verbosity()
    int get_verbosity() const { return m_verbosity; }

    /// Solve the problem using an initial guess that is based on the bounds
    /// on the variables.
    OptimalControlSolution solve() const;
    /// Solve the problem using the provided initial guess. See
    /// OptimalControlProblem::set_state_guess() and
    /// OptimalControlProblem::set_control_guess() for help with
    /// creating an initial guess.
    ///
    /// Example:
    /// @code
    /// auto ocp = std::make_shared<MyOCP>();
    /// ...
    /// OptimalControlIterate guess;
    /// guess.time.setLinSpaced(N, 0, 1);
    /// ocp->set_state_guess(guess, "x", RowVectorXd::LinSpaced(N, 0, 1));
    /// ...
    /// OptimalControlSolution solution = dircol.solve(guess);
    /// @endcode
    ///
    /// The guess will be linearly interpolated to have the requested number of
    /// mesh points.
    ///
    /// If initial_guess is empty (see OptimalControlIterate::empty()), then
    /// the no-argument solve() (above) is called.
    /// TODO right now, initial_guess.time MUST have equally-spaced intervals.
    // TODO make it even easier to create an initial guess; e.g., creating a
    // guess template.
    OptimalControlSolution solve(const OptimalControlIterate& initial_guess)
            const;

    /// Print the value of constraint vector for the given iterate. This is
    /// helpful for troubleshooting why a problem may be infeasible.
    /// This function will try to give meaningful names to the
    /// elements of the constraint vector.
    void print_constraint_values(const OptimalControlIterate& vars,
                                 std::ostream& stream = std::cout) const;
private:
    OptimalControlSolution solve_internal(Eigen::VectorXd& variables) const;
    std::shared_ptr<const OCProblem> m_ocproblem;
    // TODO perhaps ideally DirectCollocationSolver would not be templated?
    std::unique_ptr<transcription::Base<T>> m_transcription;
    std::unique_ptr<OptimizationSolver> m_optsolver;

    int m_verbosity = 1;
};

} // namespace tropter

#endif // TROPTER_DIRECTCOLLOCATION_H
