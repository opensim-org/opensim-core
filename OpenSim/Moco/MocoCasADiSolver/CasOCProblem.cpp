/* -------------------------------------------------------------------------- *
 * OpenSim Moco: CasOCProblem.cpp                                             *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2018 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include "CasOCProblem.h"

#include <OpenSim/Moco/MocoUtilities.h>
#include "CasOCTranscription.h"
#include "CasOCTrapezoidal.h"

// Shhh...we shouldn't depend on these but MocoTrajectory has a handy resample()
// function.
#include <OpenSim/Moco/MocoTrajectory.h>
#include "MocoCasOCProblem.h"

using OpenSim::Exception;

namespace CasOC {

Iterate Iterate::resample(const casadi::DM& newTimes,
                          bool appendProjectionStates = false) const {
    // Since we are converting to a MocoTrajectory and immediately converting 
    // back to a CasOC::Iterate after resampling, we do not need to provide 
    // Input control indexes, even if they are present in the MocoProblem.
    auto mocoTraj = OpenSim::convertToMocoTrajectory(*this);
    auto simtkNewTimes = OpenSim::convertToSimTKVector(newTimes);
    mocoTraj.resample(simtkNewTimes);
    return OpenSim::convertToCasOCIterate(mocoTraj, mocoTraj.getSlackNames(),
            appendProjectionStates);
}

std::vector<std::string>
Problem::createKinematicConstraintEquationNamesImpl() const {
    std::vector<std::string> names(getNumKinematicConstraintEquations());
    for (int i = 0; i < getNumKinematicConstraintEquations(); ++i) {
        names[i] = fmt::format("kinematic_constraint_{:03d}", i);
    }
    return names;
}

} // namespace CasOC
