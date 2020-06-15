#ifndef TROPTER_TROPTER_H
#define TROPTER_TROPTER_H
// ----------------------------------------------------------------------------
// tropter: tropter.h
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

#include "common.h"
#include "utilities.h"
#include "Exception.h"
#include "EigenUtilities.h"
#include "SparsityPattern.h"

#include "tropter/optimization/AbstractProblem.h"
#include "tropter/optimization/Problem.h"
#include "tropter/optimization/Solver.h"
#include "optimization/SNOPTSolver.h"
#include "optimization/IPOPTSolver.h"

#include "tropter/optimalcontrol/Iterate.h"
#include "tropter/optimalcontrol/Problem.h"
#include "optimalcontrol/DirectCollocation.h"

#include "optimalcontrol/transcription/Trapezoidal.h"
#include "optimalcontrol/transcription/HermiteSimpson.h"

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
