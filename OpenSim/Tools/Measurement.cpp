/* -------------------------------------------------------------------------- *
 *                         OpenSim:  Measurement.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Peter Loan                                                      *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "Measurement.h"

#include "OpenSim/Common/Logger.h"
#include "OpenSim/Common/Scale.h"
#include "OpenSim/Common/ScaleSet.h"
#include "OpenSim/Common/XMLDocument.h"
#include "OpenSim/Simulation/Model/BodyScale.h"
#include "OpenSim/Simulation/Model/BodyScaleSet.h"
#include "OpenSim/Tools/MarkerPair.h"
#include "OpenSim/Tools/MarkerPairSet.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Vec3;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
Measurement::Measurement() { constructProperties(); }

//_____________________________________________________________________________
/**
 * Destructor.
 */
Measurement::~Measurement()
{
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void Measurement::constructProperties() {
    constructProperty_apply(true);

    constructProperty_MarkerPairSet(MarkerPairSet());
    constructProperty_BodyScaleSet(BodyScaleSet());
}

//_____________________________________________________________________________
/**
 * Register the types used by Measurement.
 */
void Measurement::registerTypes()
{
    Object::registerType(MarkerPair());
    Object::registerType(BodyScale());
}

//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
int Measurement::getNumMarkerPairs() const {
    return get_MarkerPairSet().getSize();
}
const MarkerPair& Measurement::getMarkerPair(int aIndex) const {
    return get_MarkerPairSet()[aIndex];
}
/* Apply a scale factor to a scale set, according to the elements of
 * the Measurement's BodyScaleSet.
 */
//_____________________________________________________________________________
/**
 * Apply a scale factor to a scale set, according to the elements of
 * the Measurement's _bodyScaleSet.
 *
 * @param aFactor the scale factor to apply
 * @param aScaleSet the set of scale factors to modify
 */
void Measurement::applyScaleFactor(double aFactor, ScaleSet& aScaleSet) const {
    for (int i = 0; i < get_BodyScaleSet().getSize(); i++) {
        const auto& bodyScale = get_BodyScaleSet()[i];
        const string& bodyName = bodyScale.getName();
        const auto& axisNames = bodyScale.getProperty_axes();
        for (int j = 0; j < aScaleSet.getSize(); j++) {
            if (aScaleSet[j].getSegmentName() == bodyName)
            {
                auto& factors = aScaleSet[j].upd_scales();
                for (int k = 0; k < axisNames.size(); k++) {
                    if (axisNames[k] == "x" || axisNames[k] == "X")
                        factors[0] = aFactor;
                    else if (axisNames[k] == "y" || axisNames[k] == "Y")
                        factors[1] = aFactor;
                    else if (axisNames[k] == "z" || axisNames[k] == "Z")
                        factors[2] = aFactor;
                }
            }
        }
    }
}
