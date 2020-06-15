// ----------------------------------------------------------------------------
// tropter::optimization::Solver.cpp
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
#include "Problem.h"

#include <tropter/SparsityPattern.h>
#include <tropter/Exception.hpp>

using Eigen::VectorXd;

using namespace tropter;
using namespace tropter::optimization;

Solver::Solver(
        const AbstractProblem& problem)
        : m_problem(problem.make_decorator()) {}

void Solver::set_verbosity(int verbosity) {
    TROPTER_VALUECHECK(verbosity == 0 || verbosity == 1,
            "verbosity", verbosity, "0 or 1");
    m_verbosity = verbosity;
    m_problem->set_verbosity(verbosity);
}
int Solver::get_verbosity() const {
    return m_verbosity;
}

void Solver::set_max_iterations(Optional<int> v) {
    TROPTER_VALUECHECK(!v || *v > 0, "max_iterations", *v, "positive");
    m_max_iterations = v;
}
Optional<int> Solver::get_max_iterations() const {
    return m_max_iterations;
}

void Solver::set_convergence_tolerance(Optional<double> v) {
    TROPTER_VALUECHECK(!v || *v > 0, "convergence_tolerance", *v, "positive");
    m_convergence_tolerance = v;
}
Optional<double> Solver::get_convergence_tolerance() const {
    return m_convergence_tolerance;
}
void Solver::set_constraint_tolerance(Optional<double> v) {
    TROPTER_VALUECHECK(!v || *v > 0, "constraint_tolerance", *v, "positive");
    m_constraint_tolerance = v;
}
Optional<double> Solver::get_constraint_tolerance() const {
    return m_constraint_tolerance;
}

void Solver::set_jacobian_approximation(std::string v) {
    m_jacobian_approximation = std::move(v);
}
const std::string& Solver::get_jacobian_approximation() const {
    return m_jacobian_approximation;
}
void Solver::set_hessian_approximation(Optional<std::string> v) {
    m_hessian_approximation = v;
}
Optional<std::string> Solver::get_hessian_approximation() const {
    return m_hessian_approximation;
}
void Solver::set_sparsity_detection(std::string v) {
    TROPTER_VALUECHECK(v == "random" || v == "initial-guess",
            "sparsity_detection", v, "'random' or 'initial-guess'");
    m_sparsity_detection = std::move(v);
}
const std::string& Solver::get_sparsity_detection() const {
    return m_sparsity_detection;
}

void Solver::set_findiff_hessian_mode(std::string v) {
    m_problem->set_findiff_hessian_mode(std::move(v));
}
void Solver::set_findiff_hessian_step_size(double v) {
    m_problem->set_findiff_hessian_step_size(v);
}

void Solver::print_option_values(std::ostream& stream) const {
    const std::string unset("<unset>");
    stream << "optimization::Solver option values:\n";
    // Print non-advanced options first.
    stream << "  max iterations: ";
    if (m_max_iterations) stream << m_max_iterations.value();
    else                  stream << unset;
    stream << "\n";

    stream << "  convergence tolerance: ";
    if (m_convergence_tolerance) stream << m_convergence_tolerance.value();
    else                         stream << unset;
    stream << "\n";

    stream << "  constraint tolerance: ";
    if (m_constraint_tolerance) stream << m_constraint_tolerance.value();
    else                        stream << unset;
    stream << "\n";

    stream << "  hessian_approximation: ";
    if (m_hessian_approximation) stream << m_hessian_approximation.value();
    else                         stream << unset;
    stream << "\n";

    std::vector<std::string> available_options_string;
    std::vector<std::string> available_options_int;
    std::vector<std::string> available_options_real;
    get_available_options(available_options_string,
            available_options_int, available_options_real);

    auto contains = [](const std::vector<std::string>& v,
            const std::string& n) {
        return std::find(v.begin(), v.end(), n) != v.end();
    };

    // Print advanced options.
    const std::string unrecognized(" (unrecognized)");
    for (const auto& opt : m_advanced_options_string) {
        stream << "  [string] " << opt.first;
        if (!contains(available_options_string, opt.first))
            stream << unrecognized;
        stream << ": " << opt.second.value_or(unset);
    }
    for (const auto& opt : m_advanced_options_int) {
        stream << "  [int] " << opt.first;
        if (!contains(available_options_int, opt.first))
            stream << unrecognized;
        stream << ": ";
        if (opt.second) stream << opt.second.value();
        else            stream << unset;
        stream << "\n";
    }
    for (const auto& opt : m_advanced_options_real) {
        stream << "  [real] " << opt.first;
        if (!contains(available_options_real, opt.first))
            stream << unrecognized;
        stream << ": ";
        if (opt.second) stream << opt.second.value();
        else            stream << unset;
        stream << "\n";
    }
    stream << std::flush;
}

void Solver::set_advanced_option_string(const std::string& name,
        const std::string& value) {
    m_advanced_options_string[name] = value;
}
void Solver::set_advanced_option_int(const std::string& name,
        int value) {
    m_advanced_options_int[name] = value;
}
void Solver::set_advanced_option_real(const std::string& name,
        double value) {
    m_advanced_options_real[name] = value;
}

Solution
Solver::optimize(const Eigen::VectorXd& variables) const
{
    // if (m_verbosity > 0) print_option_values();
    // IPOPT can print the option values for us.

    m_problem->validate();
    TROPTER_THROW_IF(variables.size() != m_problem->get_num_variables(),
            "Expected guess to have %i elements, but it has %i elements.",
            m_problem->get_num_variables(), variables.size() );
    return optimize_impl(variables);
}

Solution
Solver::optimize() const {
    m_problem->validate();
    return optimize_impl(m_problem->make_initial_guess_from_bounds());
}

void Solver::calc_sparsity(const Eigen::VectorXd guess,
        SparsityCoordinates& jacobian_sparsity,
        bool provide_hessian_sparsity,
        SparsityCoordinates& hessian_sparsity
) const {
    VectorXd variables_random;
    const VectorXd* variables_for_sparsity = nullptr;
    if (get_sparsity_detection() == "initial-guess") {
        variables_for_sparsity = &guess;
    } else if (get_sparsity_detection() == "random") {
        variables_random = m_problem->make_random_iterate_within_bounds();
        variables_for_sparsity = &variables_random;
    }
    assert(variables_for_sparsity);
    m_problem->calc_sparsity(*variables_for_sparsity, jacobian_sparsity,
            provide_hessian_sparsity, hessian_sparsity);

    TROPTER_THROW_IF(!provide_hessian_sparsity && (
            hessian_sparsity.row.size() || hessian_sparsity.col.size()),
            "Hessian sparsity not requested but was provided.");
}




