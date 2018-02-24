#ifndef TROPTER_OPTIMIZATION_IPOPTSOLVER_H
#define TROPTER_OPTIMIZATION_IPOPTSOLVER_H
// ----------------------------------------------------------------------------
// tropter: IPOPTSolver.h
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
/// Here is how the IPOPTSolver applies the options in OptimizationSolver.
/// - **convergence_tolerance**: This sets the following IPOPT options:
///   - "tol",
///   - "dual_inf_tol",
///   - "compl_inf_tol",
///   - "acceptable_tol"
///   - "acceptable_dual_inf_tol"
///   - "acceptable_compl_inf_tol"
/// - **constraint_tolerance**: This sets the following IPOPT options:
///   - constr_viol_tol
///   - acceptable_constr_viol_tol
/// If you need more fine-grained control, you can use
/// OptimizationSolver::set_advanced_option_real().
///
/// @ingroup optimization
class IPOPTSolver : public Solver {
public:
    // TODO this means the IPOPTSolver *would* get access to the Problem,
    // and we don't want that.
    IPOPTSolver(const AbstractProblem& problem) : Solver(problem) {}
    // TODO explain what happens if initial guess is omitted.
    // TODO cannot use temporary.
    static void print_available_options();
protected:
    Solution optimize_impl(const Eigen::VectorXd& guess) const override;
    void get_available_options(
            std::vector<std::string>&, std::vector<std::string>&,
            std::vector<std::string>&) const override;
    void print_available_options_impl() const override {
        IPOPTSolver::print_available_options();
    }
private:
    class TNLP;
};

} // namespace optimization
} // namespace tropter

#endif // TROPTER_OPTIMIZATION_IPOPTSOLVER_H
