// ----------------------------------------------------------------------------
// tropter: OptimizationSolver.cpp
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

#include "OptimizationSolver.h"
#include "OptimizationProblem.h"

#include <tropter/Exception.hpp>

using Eigen::VectorXd;

using namespace tropter;

OptimizationSolver::OptimizationSolver(
        const AbstractOptimizationProblem& problem)
        : m_problem(problem.make_decorator()) {}

int OptimizationSolver::get_verbosity() const {
    return m_verbosity;
}
void OptimizationSolver::set_verbosity(int verbosity) {
    TROPTER_VALUECHECK(verbosity == 0 || verbosity == 1,
            "verbosity", verbosity, "0 or 1");
    m_verbosity = verbosity;
    m_problem->set_verbosity(verbosity);
}

Optional<int> OptimizationSolver::get_max_iterations() const {
    return m_max_iterations;
}
void OptimizationSolver::set_max_iterations(int max_iterations) {
    TROPTER_VALUECHECK(max_iterations > 0,
            "max_iterations", max_iterations, "positive");
    m_max_iterations = max_iterations;
}

Optional<std::string> OptimizationSolver::get_hessian_approximation() const {
    return m_hessian_approximation;
}
void OptimizationSolver::set_hessian_approximation(const std::string& value) {
    m_hessian_approximation = value;
}
void OptimizationSolver::set_findiff_hessian_mode(const std::string& setting) {
    m_problem->set_findiff_hessian_mode(setting);
}
void OptimizationSolver::set_findiff_hessian_step_size(double setting) {
    m_problem->set_findiff_hessian_step_size(setting);
}

void OptimizationSolver::print_option_values(std::ostream& stream) const {
    stream << "OptimizationSolver option values:\n";
    // Print non-advanced options first.
    stream << "  max iterations: ";
    if (m_max_iterations) stream << m_max_iterations.value();
    else                  stream << "<unset>";
    stream << "\n";

    stream << "  hessian_approximation: ";
    if (m_hessian_approximation) stream << m_hessian_approximation.value();
    else                         stream << "<unset>";
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
    for (const auto& opt : m_advanced_options_string) {
        stream << "  [string] " << opt.first;
        if (!contains(available_options_string, opt.first))
            stream << " (unrecognized)";
        stream << ": " << opt.second.value_or("<unset>");
    }
    for (const auto& opt : m_advanced_options_int) {
        stream << "  [int] " << opt.first;
        if (!contains(available_options_int, opt.first))
            stream << " (unrecognized)";
        if (opt.second) stream << opt.second.value();
        else            stream << "<unset>";
        stream << "\n";
    }
    for (const auto& opt : m_advanced_options_real) {
        stream << "  [real] " << opt.first;
        if (!contains(available_options_real, opt.first))
            stream << " (unrecognized)";
        if (opt.second) stream << opt.second.value();
        else            stream << "<unset>";
        stream << "\n";
    }
    stream << std::flush;
}

void OptimizationSolver::set_advanced_option_string(const std::string& name,
        const std::string& value) {
    m_advanced_options_string[name] = value;
}
void OptimizationSolver::set_advanced_option_int(const std::string& name,
        int value) {
    m_advanced_options_int[name] = value;
}
void OptimizationSolver::set_advanced_option_real(const std::string& name,
        double value) {
    m_advanced_options_real[name] = value;
}

OptimizationSolution
OptimizationSolver::optimize(const Eigen::VectorXd& variables) const
{
    if (m_verbosity > 0) print_option_values();
    // If the user did not provide an initial guess, then we choose
    // the initial guess based on the bounds.
    //if (variables.size() == 0) {
    //    variables = m_problem->make_initial_guess_from_bounds();
    //} // else TODO make sure variables has the correct size.
    return optimize_impl(variables.size() == 0
                         ? m_problem->make_initial_guess_from_bounds()
                         : variables);
}
