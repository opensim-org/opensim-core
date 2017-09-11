#ifndef TROPTER_OPTIMALCONTROLPROBLEM_HPP
#define TROPTER_OPTIMALCONTROLPROBLEM_HPP

#include "OptimalControlProblem.h"

namespace tropter {


template<typename T>
void OptimalControlProblem<T>::print_description() const {
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
            << this->num_path_constraints() << ")" << endl;
    for (const auto& info : m_path_constraint_infos) {
        cout << "  " << info.name << ". bounds: ["
                << info.bounds.lower << ", "
                << info.bounds.upper << "]"
                << endl;
    }
}

template<typename T>
void OptimalControlProblem<T>::
initialize_on_mesh(const Eigen::VectorXd&) const
{}

template<typename T>
void OptimalControlProblem<T>::
dynamics(const VectorX<T>&, const VectorX<T>&, Eigen::Ref<VectorX<T>>) const
{}

template<typename T>
void OptimalControlProblem<T>::
path_constraints(unsigned, const T&, const VectorX <T>&, const VectorX <T>&,
        Eigen::Ref<VectorX<T>>) const
{}

template<typename T>
void OptimalControlProblem<T>::
endpoint_cost(const T&, const VectorX<T>&, T&) const
{}

template<typename T>
void OptimalControlProblem<T>::
integral_cost(const T&, const VectorX<T>&, const VectorX<T>&, T&) const
{}

template<typename T>
void OptimalControlProblem<T>::
set_state_guess(OptimalControlIterate& guess,
        const std::string& name,
        const Eigen::VectorXd& value) {
    // Check for errors.
    if (guess.time.size() == 0) {
        throw std::runtime_error("[tropter] guess.time is empty.");
    }
    if (value.size() != guess.time.size()) {
        throw std::runtime_error("[tropter] Expected value to have " +
                std::to_string(guess.time.size()) + " elements, but it has"
                " " + std::to_string(value.size()) + ".");
    }
    if (guess.states.rows() == 0) {
        guess.states.resize(m_state_infos.size(), guess.time.size());
    } else if (size_t(guess.states.rows()) != m_state_infos.size() ||
            guess.states.cols() != guess.time.size()) {
        throw std::runtime_error("[tropter] Expected guess.states to have "
                "dimensions " + std::to_string(m_state_infos.size()) + " x "
                + std::to_string(guess.time.size()) + ", but dimensions are "
                + std::to_string(guess.states.rows()) + " x "
                + std::to_string(guess.states.cols()) + ".");
    }

    // Find the state index.
    size_t state_index = 0;
    // TODO store state infos in a map.
    for (const auto& info : m_state_infos) {
        if (info.name == name) break;
        state_index++;
    }
    if (state_index == m_state_infos.size()) {
        throw std::runtime_error(
                "[tropter] State " + name + " does not exist.");
    }

    // Set the guess.
    guess.states.row(state_index) = value;
}

template<typename T>
void OptimalControlProblem<T>::
set_control_guess(OptimalControlIterate& guess,
        const std::string& name,
        const Eigen::VectorXd& value)
{
    // Check for errors.
    if (guess.time.size() == 0) {
        throw std::runtime_error("[tropter] guess.time is empty.");
    }
    if (value.size() != guess.time.size()) {
        throw std::runtime_error("[tropter] Expected value to have " +
                std::to_string(guess.time.size()) + " elements, but it has"
                " " + std::to_string(value.size()) + ".");
    }
    if (guess.controls.rows() == 0) {
        guess.controls.resize(m_control_infos.size(), guess.time.size());
    }
    else if (size_t(guess.controls.rows()) != m_control_infos.size() ||
            guess.controls.cols() != guess.time.size()) {
        throw std::runtime_error("[tropter] Expected guess.controls to have "
                "dimensions " + std::to_string(m_control_infos.size()) + " x "
                + std::to_string(guess.time.size()) + ", but dimensions are "
                + std::to_string(guess.controls.rows()) + " x "
                + std::to_string(guess.controls.cols()) + ".");
    }
    // Find the control index.
    size_t control_index = 0;
    // TODO store control infos in a map.
    for (const auto& info : m_control_infos) {
        if (info.name == name) break;
        control_index++;
    }
    if (control_index == m_control_infos.size()) {
        throw std::runtime_error(
                "[tropter] Control " + name + " does not exist.");
    }

    // Set the guess.
    guess.controls.row(control_index) = value;
}

template<typename T>
void OptimalControlProblem<T>::all_bounds(
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
        // TODO do not create initial/final bounds constraints if they
        // are not necessary.
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
    for (unsigned ip = 0; ip < m_path_constraint_infos.size(); ++ip) {
        const auto& info = m_path_constraint_infos[ip];
        path_constraints_lower[ip] = info.bounds.lower;
        path_constraints_upper[ip] = info.bounds.upper;
    }
}

} // namespace tropter

#endif // TROPTER_OPTIMALCONTROLPROBLEM_HPP
