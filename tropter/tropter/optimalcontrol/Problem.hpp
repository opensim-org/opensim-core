#ifndef TROPTER_OPTIMALCONTROL_PROBLEM_HPP
#define TROPTER_OPTIMALCONTROL_PROBLEM_HPP
// ----------------------------------------------------------------------------
// tropter: Problem.hpp
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

#include "Problem.h"

#include <tropter/Exception.hpp>

namespace tropter {

Bounds::Bounds(double lower_bound, double upper_bound) {
    TROPTER_THROW_IF(!std::isnan(lower_bound) && !std::isnan(upper_bound)
            && lower_bound > upper_bound,
            "Expected lower <= upper, but lower=%g, upper=%g.",
            lower_bound, upper_bound);
    lower = lower_bound;
    upper = upper_bound;
}

template<typename T>
Problem<T>::ContinuousVariableInfo::
ContinuousVariableInfo(std::string n, Bounds b,
        InitialBounds ib, FinalBounds fb) {
    TROPTER_THROW_IF(ib.is_set() && ib.lower < b.lower,
            "For variable %s, expected "
            "[initial value lower bound] >= [lower bound], but "
            "intial value lower bound=%g, lower bound=%g.",
            n, ib.lower, b.lower);
    TROPTER_THROW_IF(fb.is_set() && fb.lower < b.lower,
            "For variable %s, expected "
            "[final value lower bound] >= [lower bound], but "
            "final value lower bound=%g, lower bound=%g.",
            n, fb.lower, b.lower);
    TROPTER_THROW_IF(ib.is_set() && ib.upper > b.upper,
            "For variable %s, expected "
            "[initial value upper bound] >= [upper bound], but "
            "initial value upper bound=%g, upper bound=%g.",
            n, ib.upper, b.upper);
    TROPTER_THROW_IF(fb.is_set() && fb.upper > b.upper,
            "For variable %s, expected "
            "[final value upper bound] >= [upper bound], but "
            "final value upper bound=%g, upper bound=%g.",
            n, fb.upper, b.upper);
    name = std::move(n);
    bounds = std::move(b);
    initial_bounds = std::move(ib);
    final_bounds = std::move(fb);
}

template<typename T>
void Problem<T>::set_time(const InitialBounds& initial_time,
        const FinalBounds& final_time) {
    // TODO should we force users to set time bounds? or should we
    // default to -inf, +inf?
    // TROPTER_THROW_IF(!initial_time.is_set(),
    //         "Expected initial time bounds to be specified.");
    // TROPTER_THROW_IF(!final_time.is_set(),
    //         "Expected final time bounds to be specified.");
    m_initial_time_bounds = initial_time;
    m_final_time_bounds = final_time;
}

template<typename T>
void Problem<T>::print_description() const {
    using std::cout;
    using std::endl;
    auto print_continuous_var_info = [](
            const std::string& var_type,
            const std::vector<ContinuousVariableInfo>& infos) {
        cout << var_type << ":";
        if (!infos.empty())
            cout << " (total number: " << infos.size() << ")" << endl;
        else
            cout << " none" << endl;
        for (const auto& info : infos) {
            cout << "  " << info.name << ". bounds: ["
                    << info.bounds.lower << ", "
                    << info.bounds.upper << "] ";
            if (info.initial_bounds.is_set()) {
                cout << " initial bounds: ["
                        << info.initial_bounds.lower << ", "
                        << info.initial_bounds.upper << "].";
            }
            if (info.final_bounds.is_set()) {
                cout << " final bounds: ["
                        << info.final_bounds.lower << ", "
                        << info.final_bounds.upper << "]";
            }
            cout << endl;
        }
    };

    cout << "Description of optimal control problem "
            << m_name << ":" << endl;

    print_continuous_var_info("States", m_state_infos);
    print_continuous_var_info("Controls", m_control_infos);

    cout << "Path constraints: (total number: "
            << this->get_num_path_constraints() << ")" << endl;
    for (const auto& info : m_path_constraint_infos) {
        cout << "  " << info.name << ". bounds: ["
                << info.bounds.lower << ", "
                << info.bounds.upper << "]"
                << endl;
    }
}

template<typename T>
void Problem<T>::
initialize_on_mesh(const Eigen::VectorXd&) const
{}

template<typename T>
void Problem<T>::
initialize_on_iterate(const VectorX<T>&) const
{}

template<typename T>    
void Problem<T>::
calc_differential_algebraic_equations(const DAEInput<T>&, DAEOutput<T>) const
{}

template<typename T>
void Problem<T>::
calc_endpoint_cost(const T&, const VectorX<T>&, const VectorX<T>&, T&) const
{}

template<typename T>
void Problem<T>::
calc_integral_cost(const T&, const VectorX<T>&, const VectorX<T>&, 
        const VectorX<T>&, T&) const
{}

template<typename T>
void Problem<T>::
set_state_guess(Iterate& guess,
        const std::string& name,
        const Eigen::VectorXd& value) {
    // Check for errors.
    TROPTER_THROW_IF(guess.time.size() == 0, "guess.time is empty.");
    TROPTER_THROW_IF(value.size() != guess.time.size(),
            "Expected value to have %i elements, but it has %i elements.",
            guess.time.size(), value.size());
    if (guess.states.rows() == 0) {
        guess.states.resize(m_state_infos.size(), guess.time.size());
    } else if (size_t(guess.states.rows()) != m_state_infos.size() ||
            guess.states.cols() != guess.time.size()) {
        TROPTER_THROW("Expected guess.states to have dimensions %i x %i "
                "but dimensions are %i x %i.",
                m_state_infos.size(), guess.time.size(), guess.states.rows(),
                guess.states.cols());
    }

    // Find the state index.
    size_t state_index = 0;
    // TODO store state infos in a map.
    for (const auto& info : m_state_infos) {
        if (info.name == name) break;
        state_index++;
    }
    TROPTER_THROW_IF(state_index == m_state_infos.size(),
            "State '%s' does not exist.", name);

    // Set the guess.
    guess.states.row(state_index) = value;
}

template<typename T>
void Problem<T>::
set_control_guess(Iterate& guess,
        const std::string& name,
        const Eigen::VectorXd& value)
{
    // Check for errors.
    TROPTER_THROW_IF(guess.time.size() == 0, "guess.time is empty.");
    TROPTER_THROW_IF(value.size() != guess.time.size(),
            "Expected value to have %i elements, but it has %i elements.",
            guess.time.size(), value.size());
    if (guess.controls.rows() == 0) {
        guess.controls.resize(m_control_infos.size(), guess.time.size());
    }
    else if (size_t(guess.controls.rows()) != m_control_infos.size() ||
            guess.controls.cols() != guess.time.size()) {
        TROPTER_THROW("Expected guess.controls to have dimensions %i x %i "
                "but dimensions are %i x %i.",
                m_control_infos.size(), guess.time.size(),
                guess.controls.rows(), guess.controls.cols());
    }
    // Find the control index.
    size_t control_index = 0;
    // TODO store control infos in a map.
    for (const auto& info : m_control_infos) {
        if (info.name == name) break;
        control_index++;
    }
    TROPTER_THROW_IF(control_index == m_control_infos.size(),
                "Control '%s' does not exist.", name);

    // Set the guess.
    guess.controls.row(control_index) = value;
}

template<typename T>
void Problem<T>::
set_parameter_guess(Iterate& guess,
        const std::string& name,
        const double& value)
{
    // Check for errors.
    if (guess.parameters.size() == 0) {
        guess.parameters.resize(m_parameter_infos.size());
    }
    else if (guess.parameters.size() != (int)m_parameter_infos.size()) {
        TROPTER_THROW("Expected guess.parameters to have %i elements "
            "but it has %i elements.",
            m_parameter_infos.size(), guess.parameters.size());
    }
    // Find the parameter index.
    size_t parameter_index = 0;
    // TODO store parameter infos in a map.
    for (const auto& info : m_parameter_infos) {
        if (info.name == name) break;
        parameter_index++;
    }
    TROPTER_THROW_IF(parameter_index == m_parameter_infos.size(),
        "Parameter '%s' does not exist.", name);
    std::cout << "debug: " << parameter_index << std::endl;
    std::cout << "debug2: " << value << std::endl;
    // Set the guess.
    guess.parameters(parameter_index) = value;
}

template<typename T>
void Problem<T>::get_all_bounds(
        double& initial_time_lower, double& initial_time_upper,
        double& final_time_lower, double& final_time_upper,
        Eigen::Ref<Eigen::VectorXd> states_lower,
        Eigen::Ref<Eigen::VectorXd> states_upper,
        Eigen::Ref<Eigen::VectorXd> initial_states_lower,
        Eigen::Ref<Eigen::VectorXd> initial_states_upper,
        Eigen::Ref<Eigen::VectorXd> final_states_lower,
        Eigen::Ref<Eigen::VectorXd> final_states_upper,
        Eigen::Ref<Eigen::VectorXd> controls_lower,
        Eigen::Ref<Eigen::VectorXd> controls_upper,
        Eigen::Ref<Eigen::VectorXd> initial_controls_lower,
        Eigen::Ref<Eigen::VectorXd> initial_controls_upper,
        Eigen::Ref<Eigen::VectorXd> final_controls_lower,
        Eigen::Ref<Eigen::VectorXd> final_controls_upper,
        Eigen::Ref<Eigen::VectorXd> parameters_lower,
        Eigen::Ref<Eigen::VectorXd> parameters_upper,
        Eigen::Ref<Eigen::VectorXd> path_constraints_lower,
        Eigen::Ref<Eigen::VectorXd> path_constraints_upper) const {
    initial_time_lower = m_initial_time_bounds.lower;
    initial_time_upper = m_initial_time_bounds.upper;
    final_time_lower = m_final_time_bounds.lower;
    final_time_upper = m_final_time_bounds.upper;
    for (unsigned is = 0; is < m_state_infos.size(); ++is) {
        const auto& info = m_state_infos[is];
        states_lower[is]         = info.bounds.lower;
        states_upper[is]         = info.bounds.upper;
        if (info.initial_bounds.is_set()) {
            initial_states_lower[is] = info.initial_bounds.lower;
            initial_states_upper[is] = info.initial_bounds.upper;
        } else {
            initial_states_lower[is] = info.bounds.lower;
            initial_states_upper[is] = info.bounds.upper;
        }
        if (info.final_bounds.is_set()) {
            final_states_lower[is]   = info.final_bounds.lower;
            final_states_upper[is]   = info.final_bounds.upper;
        } else {
            final_states_lower[is]   = info.bounds.lower;
            final_states_upper[is]   = info.bounds.upper;
        }
    }
    for (unsigned ic = 0; ic < m_control_infos.size(); ++ic) {
        const auto& info = m_control_infos[ic];
        // TODO if (!info.bounds.is_set()), give error.
        controls_lower[ic]         = info.bounds.lower;
        controls_upper[ic]         = info.bounds.upper;
        if (info.initial_bounds.is_set()) {
            initial_controls_lower[ic] = info.initial_bounds.lower;
            initial_controls_upper[ic] = info.initial_bounds.upper;
        } else {
            initial_controls_lower[ic] = info.bounds.lower;
            initial_controls_upper[ic] = info.bounds.upper;
        }
        if (info.final_bounds.is_set()) {
            final_controls_lower[ic]   = info.final_bounds.lower;
            final_controls_upper[ic]   = info.final_bounds.upper;
        } else {
            final_controls_lower[ic]   = info.bounds.lower;
            final_controls_upper[ic]   = info.bounds.upper;
        }
    }
    for (unsigned ip = 0; ip < m_parameter_infos.size(); ++ip) {
        const auto& info = m_parameter_infos[ip];
        parameters_lower[ip] = info.bounds.lower;
        parameters_upper[ip] = info.bounds.upper;
    }
    for (unsigned ipc = 0; ipc < m_path_constraint_infos.size(); ++ipc) {
        const auto& info = m_path_constraint_infos[ipc];
        path_constraints_lower[ipc] = info.bounds.lower;
        path_constraints_upper[ipc] = info.bounds.upper;
    }
}

} // namespace tropter

#endif // TROPTER_OPTIMALCONTROL_PROBLEM_HPP
