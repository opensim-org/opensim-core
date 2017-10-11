#ifndef TROPTER_TROPTER_H
#define TROPTER_TROPTER_H

#include "common.h"
#include "Exception.h"
#include "EigenUtilities.h"

#include "optimization/AbstractOptimizationProblem.h"
#include "optimization/OptimizationProblem.h"
#include "optimization/OptimizationSolver.h"
#include "optimization/SNOPTSolver.h"
#include "optimization/IpoptSolver.h"

#include "optimalcontrol/OptimalControlIterate.h"
#include "optimalcontrol/OptimalControlProblem.h"
#include "optimalcontrol/DirectCollocation.h"

// http://www.coin-or.org/Ipopt/documentation/node23.html

// TODO create my own "NonnegativeIndex" or Count type.

// TODO use faster linear solvers from Sherlock cluster.

// TODO consider namespace opt for generic NLP stuff.


// TODO interface 0: (inheritance)
// derive from Problem, implement virtual functions
// interface 1: (composition)
// composed of Variables, Controls, Goals, etc.
/*
std::unordered_map<std::string, Goal> m_goals;
std::unordered_map<std::string
 * */

#endif // TROPTER_TROPTER_H
