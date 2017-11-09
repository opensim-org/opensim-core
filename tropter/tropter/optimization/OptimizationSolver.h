#ifndef TROPTER_OPTIMIZATIONSOLVER_H
#define TROPTER_OPTIMIZATIONSOLVER_H
// ----------------------------------------------------------------------------
// tropter: OptimizationSolver.h
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

class AbstractOptimizationProblem;

class OptimizationProblemDecorator;

struct OptimizationSolution {
    Eigen::VectorXd variables;
    double objective = std::numeric_limits<double>::quiet_NaN();
    bool success = false;
    std::string status;
};

/// @ingroup optimization
class OptimizationSolver {
public:
    /// Provide the problem to solve.
    OptimizationSolver(const AbstractOptimizationProblem& problem);
    /// Optimize the optimization problem.
    /// @param[in] guess
    ///     Initial guess to the problem, or leave empty to use a naive initial
    ///     guess based on the variables' bounds
    ///     (see OptimizationProblemProxy::make_initial_guess_from_bounds()).
    /// @returns the solution (optimal variables, objective, and the solver's
    ///     return status).
    /// @returns The value of the objective function evaluated at the solution.
    OptimizationSolution optimize(const Eigen::VectorXd& guess) const;

    /// @name Set common options
    /// @{

    /// How much information should be spewed to the console?
    /// 0: tropter and the underlying solver are silent.
    /// 1: tropter is verbose, and the underlying solver uses its default
    /// verbosity.
    void set_verbosity(int value);
    /// The maximum number of iterations the optimizer is allowed to take.
    void set_max_iterations(int setting);
    /// Whether a full Hessian should be computed or if the Hessian
    /// should be approximated from the gradient using BFGS updates.
    /// See https://www.coin-or.org/Ipopt/documentation/node53.html#SECTION0001113010000000000000
    void set_hessian_approximation(const std::string& setting);
    /// @copydoc OptimizationProblemDecorator::set_findiff_hessian_mode()
    void set_findiff_hessian_mode(const std::string& setting);
    /// @copydoc OptimizationProblemDecorator::set_findiff_hessian_step_size()
    void set_findiff_hessian_step_size(double setting);
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
    void print_option_values() const;
    /// @copydoc set_verbosity()
    int get_verbosity() const;
    /// @copydoc set_max_iterations()
    Optional<int> get_max_iterations() const;
    /// @copydoc set_hessian_approximation()
    Optional<std::string> get_hessian_approximation() const;
    /// @}

protected:
    virtual OptimizationSolution
    optimize_impl(const Eigen::VectorXd& variables) const = 0;
    virtual void get_available_options(
            std::vector<std::string>& options_string,
            std::vector<std::string>& options_int,
            std::vector<std::string>& options_real) const;
    virtual void print_available_options_impl() const {}



    std::unique_ptr<OptimizationProblemDecorator> m_problem;

    template <typename T>
    using OptionsMap = std::unordered_map<std::string, Optional<T>>;

    const OptionsMap<std::string>& get_advanced_options_string() const
    {   return m_advanced_options_string; }
    const OptionsMap<int>& get_advanced_options_int() const
    {   return m_advanced_options_int; }
    const OptionsMap<double>& get_advanced_options_real() const
    {   return m_advanced_options_real; }

private:
    int m_verbosity;
    Optional<int> m_max_iterations;
    Optional<std::string> m_hessian_approximation;

    OptionsMap<std::string> m_advanced_options_string;
    OptionsMap<int> m_advanced_options_int;
    OptionsMap<double> m_advanced_options_real;
};

inline void OptimizationSolver::get_available_options(
        std::vector<std::string>&, std::vector<std::string>&,
        std::vector<std::string>&) const {}

} // namespace tropter

#endif // TROPTER_OPTIMIZATIONSOLVER_H
