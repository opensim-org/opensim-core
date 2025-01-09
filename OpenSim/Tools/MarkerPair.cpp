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
MarkerPair::MarkerPair() :
    _markerNames(_markerNamesProp.getValueStrArray())
{
    setNull();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
MarkerPair::~MarkerPair()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aMarkerPair MarkerPair to be copied.
 */
MarkerPair::MarkerPair(const MarkerPair &aMarkerPair) :
   Object(aMarkerPair),
    _markerNames(_markerNamesProp.getValueStrArray())
{
    setNull();
    copyData(aMarkerPair);
}
//_____________________________________________________________________________
/**
 */
MarkerPair::MarkerPair(const std::string &aName1, const std::string &aName2) :
    _markerNames(_markerNamesProp.getValueStrArray())
{
    setNull();
    _markerNames.append(aName1);
    _markerNames.append(aName2);
}


void MarkerPair::copyData(const MarkerPair &aMarkerPair)
{
    _markerNames = aMarkerPair._markerNames;
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this MarkerPair to their null values.
 */
void MarkerPair::setNull()
{
    setupProperties();
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void MarkerPair::setupProperties()
{
    _markerNamesProp.setComment("Names of two markers, the distance between which is used to compute a body scale factor.");
    _markerNamesProp.setName("markers");
    _propertySet.append(&_markerNamesProp);
}

MarkerPair& MarkerPair::operator=(const MarkerPair &aMarkerPair)
{
    // BASE CLASS
    Object::operator=(aMarkerPair);

    copyData(aMarkerPair);

    return(*this);
}

void MarkerPair::getMarkerNames(string& aName1, string& aName2) const
{
    aName1 = _markerNames[0];
    aName2 = _markerNames[1];
}
