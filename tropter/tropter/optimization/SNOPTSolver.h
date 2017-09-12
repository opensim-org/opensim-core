#ifndef TROPTER_SNOPTSOLVER_H
#define TROPTER_SNOPTSOLVER_H

#include "OptimizationSolver.h"

namespace tropter {

/// @ingroup optimization
class SNOPTSolver : public OptimizationSolver {
public:
    // TODO this means the SNOPTSolver *would* get access to the Problem,
    // and we don't want that.
    SNOPTSolver(const AbstractOptimizationProblem& problem)
            : OptimizationSolver(problem) {}
    // TODO explain what happens if initial guess is omitted.
    // TODO cannot use temporary.
protected:
    double optimize_impl(Eigen::VectorXd& variables) const override;
private:
    // TODO come up with a better name; look at design patterns book?
    class TNLP;
};

} // namespace tropter

#endif // TROPTER_SNOPTSOLVER_H
