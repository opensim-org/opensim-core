#ifndef TROPTER_IPOPTSOLVER_H
#define TROPTER_IPOPTSOLVER_H

#include "OptimizationSolver.h"

namespace tropter {

class IpoptSolver : public OptimizationSolver {
public:
    // TODO this means the IpoptSolver *would* get access to the Problem,
    // and we don't want that.
    IpoptSolver(const AbstractOptimizationProblem& problem)
            : OptimizationSolver(problem) {}
    // TODO explain what happens if initial guess is omitted.
    // TODO cannot use temporary.
protected:
    double optimize_impl(Eigen::VectorXd& variables) const override;
private:
    class TNLP;
};


} // namespace tropter

#endif // TROPTER_IPOPTSOLVER_H
