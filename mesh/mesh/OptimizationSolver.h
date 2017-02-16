#ifndef MESH_OPTIMIZATIONSOLVER_H
#define MESH_OPTIMIZATIONSOLVER_H

#include "common.h"
#include <Eigen/Dense>
// TODO should be able to remove dependenc on adolc in this file.
#include <adolc/adolc.h>
#include <IpTNLP.hpp>

#include <memory>

namespace mesh {

class AbstractOptimizationProblem;

class OptimizationProblemProxy;

// TODO templatized?
class OptimizationSolver {
public:
    // TODO do not force adouble in the future.
    OptimizationSolver(const AbstractOptimizationProblem& problem);
    // TODO must be an lvalue??
    // TODO might want to change this interface.
    double optimize(Eigen::VectorXd& variables) const;
protected:
    virtual double optimize_impl(Eigen::VectorXd& variables) const = 0;
    std::shared_ptr<const OptimizationProblemProxy> m_problem;
    // TODO rename m_problem to m_proxy? m_probproxy?
//    const OptimizationProblemProxy& m_problem;
};

// TODO perhaps NewtonSolver {}; ?

} // namespace mesh

#endif // MESH_OPTIMIZATIONSOLVER_H
