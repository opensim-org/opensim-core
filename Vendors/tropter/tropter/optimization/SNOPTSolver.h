#ifndef TROPTER_OPTIMIZATION_SNOPTSOLVER_H
#define TROPTER_OPTIMIZATION_SNOPTSOLVER_H
// ----------------------------------------------------------------------------
// tropter: SNOPTSolver.h
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

#include "Solver.h"

namespace tropter {
namespace optimization {

/// Options
/// =======
/// Here is how the SNOPTSolver applies the options in OptimizationSolver.
/// - **convergence_tolerance**: Currently ignored.
/// - **constraint_tolerance**: Currently ignored.
/// @ingroup optimization
class SNOPTSolver : public Solver {
public:
    // TODO this means the SNOPTSolver *would* get access to the Problem,
    // and we don't want that.
    SNOPTSolver(const AbstractProblem& problem)
            : Solver(problem) {}
    // TODO explain what happens if initial guess is omitted.
    // TODO cannot use temporary.
protected:
    Solution
    optimize_impl(const Eigen::VectorXd& guess) const override;
private:
    // TODO come up with a better name; look at design patterns book?
    class TNLP;
};

} // namespace optimization
} // namespace tropter

#endif // TROPTER_OPTIMIZATION_SNOPTSOLVER_H
