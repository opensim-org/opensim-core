#ifndef MESH_MESH_H
#define MESH_MESH_H

#include <iostream>
#include <Eigen/Dense>
#include <IpTNLP.hpp>
#include <adolc/adolc.h>
#include "common.h"

// TODO remove.
#include "OptimizationProblem.h"
#include "OptimizationSolver.h"
#include "SNOPTSolver.h"
#include "IpoptSolver.h"

#include "OptimalControlProblem.h"
#include "DirectCollocation.h"
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

#endif // MESH_MESH_H
