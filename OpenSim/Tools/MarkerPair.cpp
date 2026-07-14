/* -------------------------------------------------------------------------- *
 *                          OpenSim:  MarkerPair.cpp                          *
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
#include "MarkerPair.h"

#include "OpenSim/Common/Array.h"

//=============================================================================
// STATICS
//=============================================================================


using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
MarkerPair::MarkerPair() { constructProperties(); }

//_____________________________________________________________________________
/**
 * Destructor.
 */
MarkerPair::~MarkerPair()
{
}

//_____________________________________________________________________________
/**
 */
MarkerPair::MarkerPair(const std::string& aName1, const std::string& aName2) {
    updProperty_markers().appendValue(aName1);
    updProperty_markers().appendValue(aName2);
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void MarkerPair::constructProperties() {
    Array<std::string> markerNames{"", ""};
    constructProperty_markers(markerNames);
}

void MarkerPair::getMarkerNames(string& aName1, string& aName2) const
{
    aName1 = get_markers(0);
    aName2 = get_markers(1);
}
