#ifndef TROPTER_OPTIMIZATION_PROBLEMDECORATOR_H
#define TROPTER_OPTIMIZATION_PROBLEMDECORATOR_H
// ----------------------------------------------------------------------------
// tropter: ProblemDecorator.h
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
#include <tropter/utilities.h>

#include "AbstractProblem.h"

namespace tropter {

struct SparsityCoordinates;

namespace optimization {

/// This class provides an interface of the OptimizationProblem to the
/// OptimizationSolvers.
/// In general, users do not use this class directly.
/// There is a derived type for each scalar type, and these derived classes
/// compute the gradient, Jacobian, and Hessian (using either finite differences
/// or automatic differentiation).
/// @ingroup optimization
// TODO alternative name: OptimizationProblemDelegate
class ProblemDecorator {
public:
    ProblemDecorator(const AbstractProblem& problem)
            :m_problem(problem) { }
    virtual ~ProblemDecorator() = default;
    unsigned get_num_variables() const
    {   return m_problem.get_num_variables(); }
    unsigned get_num_constraints() const
    {   return m_problem.get_num_constraints(); }
    const Eigen::VectorXd& get_variable_lower_bounds() const
    {   return m_problem.get_variable_lower_bounds(); }
    const Eigen::VectorXd& get_variable_upper_bounds() const
    {   return m_problem.get_variable_upper_bounds(); }
    const Eigen::VectorXd& get_constraint_lower_bounds() const
    {   return m_problem.get_constraint_lower_bounds(); }
    const Eigen::VectorXd& get_constraint_upper_bounds() const
    {   return m_problem.get_constraint_upper_bounds(); }

    std::vector<std::string> get_variable_names() const
    {   return m_problem.get_variable_names(); }

    std::vector<std::string> get_constraint_names() const
    {   return m_problem.get_constraint_names(); }

    void validate() const { m_problem.validate(); }

    /// @see AbstractOptimizationProblem::make_initial_guess_from_bounds()
    Eigen::VectorXd make_initial_guess_from_bounds() const
    {   return m_problem.make_initial_guess_from_bounds(); }
    /// @see AbstractOptimizationProblem::make_random_iterate_within_bounds()
    Eigen::VectorXd make_random_iterate_within_bounds() const
    {   return m_problem.make_random_iterate_within_bounds(); }
    /// This function determines the sparsity pattern of the Jacobian and
    /// Hessian, using the provided variables.
    /// You must call this function first before calling calc_objective(),
    /// calc_constraints(), etc.
    // TODO create a struct to hold row and col indices.
    // TODO b/c of SNOPT, want to be able to ask for sparsity separately.
    virtual void calc_sparsity(const Eigen::VectorXd& variables,
            SparsityCoordinates& jacobian_sparsity,
            bool provide_hessian_sparsity,
            SparsityCoordinates& hessian_sparsity) const = 0;
    virtual void calc_objective(unsigned num_variables, const double* variables,
            bool new_variables,
            double& obj_value) const = 0;
    virtual void calc_constraints(unsigned num_variables,
            const double* variables,
            bool new_variables,
            unsigned num_constraints, double* constr) const = 0;
    virtual void calc_gradient(unsigned num_variables, const double* variables,
            bool new_variables,
            double* grad) const = 0;
    virtual void calc_jacobian(unsigned num_variables, const double* variables,
            bool new_variables,
            unsigned num_nonzeros, double* nonzeros) const = 0;
    virtual void calc_hessian_lagrangian(
            unsigned num_variables, const double* variables, bool new_variables,
            double obj_factor,
            unsigned num_constraints, const double* lambda, bool new_lambda,
            unsigned num_nonzeros, double* nonzeros) const = 0;
    /// 0 for silent, 1 for verbose.
    void set_verbosity(int verbosity);
    /// @copydoc set_verbosity()
    int get_verbosity() const;

    /// @name Options for finite differences
    /// These options are only used when the scalar type is double.
    /// @{

    /// The finite difference step size used when approximating the Hessian.
    /// (default: 1e-5, based on [1] section 9.2.4.4).
    /// [1] Bohme TJ, Frank B. Hybrid Systems, Optimal Control and Hybrid
    /// Vehicles: Theory, Methods and Applications. Springer 2017.
    void set_findiff_hessian_step_size(double value);
    ///  - "fast": default. Reduce the number of calls to the constraint
    ///    function by using graph coloring.
    ///  - "slow": Slower mode to be used only for debugging. Each nonzero of
    ///    the Hessian of the Lagrangian is computed separately.
    void set_findiff_hessian_mode(std::string value);
    /// @copydoc set_findiff_hessian_step_size()
    double get_findiff_hessian_step_size() const;
    /// @copydoc set_findiff_hessian_mode()
    const std::string& get_findiff_hessian_mode() const;
    /// @}

protected:
    template<typename ...Types>
    void print(const std::string& format_string, Types... args) const;
private:
    const AbstractProblem& m_problem;
    int m_verbosity = 1;
    double m_findiff_hessian_step_size = 1e-5;
    std::string m_findiff_hessian_mode = "fast";
};

inline int ProblemDecorator::get_verbosity() const
{   return m_verbosity; }
inline double ProblemDecorator::get_findiff_hessian_step_size() const
{   return m_findiff_hessian_step_size; }
inline const std::string& ProblemDecorator::get_findiff_hessian_mode() const
{   return m_findiff_hessian_mode; }
template<typename ...Types>
inline void ProblemDecorator::print(
        const std::string& format_string, Types... args) const {
    if (m_verbosity)
        std::cout << "[tropter] " << format(format_string.c_str(), args...) <<
                std::endl;
}

} // namespace optimization
} // namespace tropter

#endif // TROPTER_OPTIMIZATION_PROBLEMDECORATOR_H
