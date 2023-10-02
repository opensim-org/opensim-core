#ifndef OPENSIM_LATINHYPERCUBEDESIGN_H
#define OPENSIM_LATINHYPERCUBEDESIGN_H
/* -------------------------------------------------------------------------- *
 *                      OpenSim:  LatinHypercubeDesign.h                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2023 Stanford University and the Authors                *
 * Author(s): Nicholas Bianco                                                 *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include "osimCommonDLL.h"
#include "CommonUtilities.h"

namespace OpenSim {

// A class for generating Latin hypercube designs.
class OSIMCOMMON_API LatinHypercubeDesign {
public:
    LatinHypercubeDesign() = default;

    SimTK::Matrix computeTranslationalPropagationDesign(int numSamples,
            int numVariables, SimTK::Matrix seed, int numSeedPoints);

private:
    std::string m_criterion = "maximin"; // maximin, phi_p

    double computeMaximinDistance(const SimTK::Matrix& x);

    SimTK::Matrix computeIntersiteDistanceMatrix(
            const SimTK::Matrix& x);

    double computePhiPDistanceCriterion(const SimTK::Matrix& x,
            const SimTK::Matrix& distances, int p = 50);

};

} // namespace OpenSim

#endif // OPENSIM_LATINHYPERCUBEDESIGN_H
