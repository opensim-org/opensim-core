#ifndef TROPTER_OPTIMIZATION_SOLVER_H
#define TROPTER_OPTIMIZATION_SOLVER_H
// ----------------------------------------------------------------------------
// tropter::optimization::Solver.h
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

#include <tropter/common.h>
#include <Eigen/Dense>

#include <memory>
#include <unordered_map>

namespace tropter {

struct SparsityCoordinates;

namespace optimization {

class AbstractProblem;

class ProblemDecorator;

struct Solution {
    Eigen::VectorXd variables;
    double objective = std::numeric_limits<double>::quiet_NaN();
    bool success = false;
    /// Number of solver iterations at which this solution was obtained.
    int num_iterations = -1;
    std::string status;
};

/// The OptimizationSolver class contains some generic options that are
/// common across different concrete solvers. To learn how the different
/// solves interpret these generic options, view the documentaiton for the
/// concrete solvers.
///
/// @ingroup optimization
class Solver {
public:
    /// Provide the problem to solve.
    Solver(const AbstractProblem& problem);
    virtual ~Solver() = default;
    /// Optimize the optimization problem.
    /// @param[in] guess
    ///     Initial guess to the problem; the length must match the number of
    /// variables.
    /// @returns the solution (optimal variables, objective, and the solver's
    ///     return status).
    /// @returns The value of the objective function evaluated at the solution.
    Solution optimize(const Eigen::VectorXd& guess) const;
    /// Optimize the optimization problem, without providing your own initial
    /// guess.
    /// The guess will be based on the variables' bounds (see
    /// OptimizationProblemProxy::make_initial_guess_from_bounds()).
    /// @returns The value of the objective function evaluated at the solution.
    Solution optimize() const;

    /// @name Set common options
    /// @{

    /// How much information should be spewed to the console?
    /// 0: tropter and the underlying solver are silent.
    /// 1: tropter is verbose, and the underlying solver uses its default
    /// verbosity.
    void set_verbosity(int value);
    /// The maximum number of iterations the optimizer is allowed to take.
    void set_max_iterations(Optional<int> v);
    /// The tolerance used to determine if the objective is minimized.
    void set_convergence_tolerance(Optional<double> value);
    /// The tolerance used to determine if the constraints are satisfied.
    void set_constraint_tolerance(Optional<double> value);
    /// Whether the Jacobian is calculated by tropter or by IPOPT's own finite
    /// differencing.
    /// See https://www.coin-or.org/Bonmin/option_pages/options_list_ipopt.html
    /// @note This setting currently takes effect only when using IPOPT.
    void set_jacobian_approximation(std::string v);
    /// Whether a full Hessian should be computed or if the Hessian
    /// should be approximated from the gradient using BFGS updates.
    /// See https://www.coin-or.org/Ipopt/documentation/node53.html#SECTION0001113010000000000000
    /// @note This setting currently takes effects only when using IPOPT.
    void set_hessian_approximation(Optional<std::string> v);
    void set_hessian_approximation(const std::string& v)
    {   set_hessian_approximation(nonstd::optional_lite::make_optional(v)); }

    /// What point should be used to detect the sparsity of the Jacobian and
    /// Hessian?
    ///   - "initial-guess": perturb about the initial guess (default).
    ///   - "random": perturb about a random point.
    void set_sparsity_detection(std::string v);

    /// @copydoc ProblemDecorator::set_findiff_hessian_mode()
    void set_findiff_hessian_mode(std::string v);
    /// @copydoc ProblemDecorator::set_findiff_hessian_step_size()
    void set_findiff_hessian_step_size(double value);
    /// @}

    /// @name Set solver-specific advanced options.
    /// @{

    /// Print a list of the specific solver's available options (not
    /// necessarily the ones you've set).
    void print_available_options() const
    {   print_available_options_impl(); }
    /// If the name provided here maps onto the same option as one of the
    /// non-advanced options above (e.g., set_max_iterations), then the value
    /// set via set_option() overrides.
    void set_advanced_option_string(const std::string& name,
            const std::string& value);
    /// If the name provided here maps onto the same option as one of the
    /// non-advanced options above (e.g., set_max_iterations), then the value
    /// set via set_option() overrides.
    void set_advanced_option_int(const std::string& name, int value);
    /// If the name provided here maps onto the same option as one of the
    /// non-advanced options above (e.g., set_max_iterations), then the value
    /// set via set_option() overrides.
    void set_advanced_option_real(const std::string& name, double value);
    /// @}

    /// @name Access values of options
    /// @{

    /// Print a list of options you've set and their current values.
    void print_option_values(std::ostream& = std::cout) const;
    /// @copydoc set_verbosity()
    int get_verbosity() const;
    /// @copydoc set_max_iterations()
    Optional<int> get_max_iterations() const;
    /// @copydoc set_convergence_tolerance()
    Optional<double> get_convergence_tolerance() const;
    /// @copydoc set_constraint_tolerance()
    Optional<double> get_constraint_tolerance() const;
    /// @copydoc set_jacobian_approximation()
    const std::string& get_jacobian_approximation() const;
    /// @copydoc set_hessian_approximation()
    Optional<std::string> get_hessian_approximation() const;
    const std::string& get_sparsity_detection() const;
    /// @}

protected:
    virtual Solution optimize_impl(const Eigen::VectorXd& guess) const = 0;
    virtual void get_available_options(
            std::vector<std::string>& options_string,
            std::vector<std::string>& options_int,
            std::vector<std::string>& options_real) const;
    virtual void print_available_options_impl() const {}

    /// @name Services to derived classes.
    /// @{

    /// This calls the decorator's calc_sparsity() function with the appropriate
    /// handling of the sparsity_detection setting. The provided guess may not
    /// be used.
    /// @throws if provide_hessian_indices is false but the decorator provides
    ///         Hessian indices.
    void calc_sparsity(const Eigen::VectorXd guess,
            SparsityCoordinates& jacobian_sparsity,
            bool provide_hessian_sparsity,
            SparsityCoordinates& hessian_sparsity) const;
    /// @}




    std::unique_ptr<ProblemDecorator> m_problem;

    template <typename T>
    using OptionsMap = std::unordered_map<std::string, Optional<T>>;

    const OptionsMap<std::string>& get_advanced_options_string() const
    {   return m_advanced_options_string; }
    const OptionsMap<int>& get_advanced_options_int() const
    {   return m_advanced_options_int; }
    const OptionsMap<double>& get_advanced_options_real() const
    {   return m_advanced_options_real; }

private:
    int m_verbosity = 1;
    Optional<int> m_max_iterations;
    Optional<double> m_convergence_tolerance;
    Optional<double> m_constraint_tolerance;
    std::string m_jacobian_approximation = "exact";
    Optional<std::string> m_hessian_approximation;
    std::string m_sparsity_detection = "initial-guess";

    OptionsMap<std::string> m_advanced_options_string;
    OptionsMap<int> m_advanced_options_int;
    OptionsMap<double> m_advanced_options_real;
};

inline void Solver::get_available_options(
        std::vector<std::string>&, std::vector<std::string>&,
        std::vector<std::string>&) const {}

} // namespace optimization
} // namespace tropter

#endif // TROPTER_OPTIMIZATION_SOLVER_H
