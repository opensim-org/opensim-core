#include "OptimizationProblem.hpp"

namespace tropter {

Eigen::VectorXd OptimizationProblemDecorator::initial_guess_from_bounds() const
{
    const auto& lower = variable_lower_bounds();
    const auto& upper = variable_upper_bounds();
    assert(lower.size() == upper.size());
    Eigen::VectorXd guess(lower.size());
    const auto inf = std::numeric_limits<double>::infinity();
    for (Eigen::Index i = 0; i < lower.size(); ++i) {
        if (lower[i] != -inf && upper[i] != inf) {
            guess[i] = 0.5 * (upper[i] + lower[i]);
        }
        else if (lower[i] != -inf) guess[i] = lower[i];
        else if (upper[i] !=  inf) guess[i] = upper[i];
        else guess[i] = 0;
    }
    return guess;
}

// Explicit instantiation.
template class OptimizationProblem<double>;
template class OptimizationProblem<adouble>;
// TODO extern to avoid implicit instantiation and improve compile time?

} // namespace tropter
