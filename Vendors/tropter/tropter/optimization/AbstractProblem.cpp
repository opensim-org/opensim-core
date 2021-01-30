// ----------------------------------------------------------------------------
// tropter: AbstractProblem.cpp
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

#include "AbstractProblem.h"

#include <tropter/Exception.hpp>

using namespace tropter::optimization;

void AbstractProblem::validate() const {
    TROPTER_THROW_IF(m_num_variables != m_variable_lower_bounds.size() ||
            m_num_variables != m_variable_upper_bounds.size(),
            "Number of variables does not match number of variable bounds; "
            "did you call set_variable_bounds()?");
    TROPTER_THROW_IF(m_num_constraints != m_constraint_lower_bounds.size() ||
            m_num_constraints != m_constraint_upper_bounds.size(),
            "Number of constraints does not match number of constraint bounds; "
            "did you call set_constraint_bounds()?");
}
