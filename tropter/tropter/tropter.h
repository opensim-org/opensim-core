#ifndef TROPTER_TROPTER_H
#define TROPTER_TROPTER_H

#include "common.h"

// TODO remove.
#include "optimization/AbstractOptimizationProblem.h"
#include "optimization/OptimizationProblem.h"
#include "optimization/OptimizationSolver.h"
#include "optimization/SNOPTSolver.h"
#include "optimization/IpoptSolver.h"

#include "optimalcontrol/OptimalControlIterate.h"
#include "optimalcontrol/OptimalControlProblem.h"
#include "optimalcontrol/DirectCollocation.h"
// TODO should not have using declarations in a header file.

// http://www.coin-or.org/Ipopt/documentation/node23.html

// TODO create my own "NonnegativeIndex" or Count type.

// TODO use faster linear solvers from Sherlock cluster.

// TODO want to abstract optimization problem away from IPOPT.


// TODO provide a C interface?

// TODO consider namespace opt for generic NLP stuff.


// TODO templatize Problem.

// TODO interface 0: (inheritance)
// derive from Problem, implement virtual functions
// interface 1: (composition)
// composed of Variables, Controls, Goals, etc.
/*
std::unordered_map<std::string, Goal> m_goals;
std::unordered_map<std::string
 * */

#endif // TROPTER_TROPTER_H
