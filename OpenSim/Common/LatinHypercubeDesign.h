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
    // CONSTRUCTION
    LatinHypercubeDesign() = default;
    LatinHypercubeDesign(int numSamples, int numVariables) :
        m_numSamples(numSamples), m_numVariables(numVariables) {}

    LatinHypercubeDesign(const LatinHypercubeDesign&) = default;
    LatinHypercubeDesign(LatinHypercubeDesign&&) = default;
    LatinHypercubeDesign& operator=(const LatinHypercubeDesign&) = default;
    LatinHypercubeDesign& operator=(LatinHypercubeDesign&&) = default;

    // GET AND SET
    /// num samples
    void setNumSamples(int numSamples) {
        m_numSamples = numSamples;
    }
    int getNumSamples() const {
        return m_numSamples;
    }

    /// num variables
    void setNumVariables(int numVariables) {
        m_numVariables = numVariables;
    }
    int getNumVariables() const {
        return m_numVariables;
    }

    /// distance criterion
    void setDistanceCriterion(std::string distanceCriterion) {
        m_distanceCriterion = std::move(distanceCriterion);
        m_useMaximinDistanceCriterion = m_distanceCriterion == "maximin";
    }
    const std::string& getDistanceCriterion() const {
        return m_distanceCriterion;
    }

    /// tplhs design
    SimTK::Matrix generateTranslationalPropagationDesign(
            int numSeedPoints = -1) const;



    SimTK::Matrix computeTranslationalPropagationDesign(
            int numSamples, SimTK::Matrix seed) const;

private:

    double computeDistanceCriterion(const SimTK::Matrix& design) const;
    double computeMaximinDistanceCriterion(const SimTK::Matrix& design) const;
    double computePhiDistanceCriterion(const SimTK::Matrix& design) const;

    void checkConfiguration() const;

    int m_numSamples = -1;
    int m_numVariables = -1;
    std::string m_distanceCriterion = "maximin";
    bool m_useMaximinDistanceCriterion = true;
    int m_phiDistanceExponent = 25;

};

} // namespace OpenSim

#endif // OPENSIM_LATINHYPERCUBEDESIGN_H
