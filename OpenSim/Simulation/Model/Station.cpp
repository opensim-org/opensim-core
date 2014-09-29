/* -------------------------------------------------------------------------- *
 *                            OpenSim:  Station.cpp                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ayman Habib                                                     *
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
#include "Station.h"
#include "Model.h"

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
Station::Station() :
   ModelComponent()
{
	setNull();
	constructInfrastructure();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
Station::~Station()
{
}

//_____________________________________________________________________________
/**
* Set the data members of this Station to their null values.
*/
void Station::setNull()
{
	setAuthors("Ayman Habib");
}

//_____________________________________________________________________________
/**
* Connect properties to local pointers.
*/
void Station::constructProperties()
{
	//Default location
	SimTK::Vec3 origin(0.0, 0.0, 0.0);
	// Location in Body 
	constructProperty_location(origin);
}


void Station::constructConnectors()
{
    constructConnector<RigidFrame>("reference_frame");
}

/**
 * Return the reference frame with respect to which this station is defined
 *
*/
const RigidFrame& Station::getReferenceFrame() const
{
	return getConnector<RigidFrame>("reference_frame").getConnectee();
}
/**
 * setReferenceFrame sets the "reference_frame" connection
 *
 * @param aFrame a frame to be used as reference. 
 * 
 */
// TODO: Connection is based on name so it may make more sense to pass in name instead
// TODO: Not clear what to do when connection is re-established or who would trigger it

void Station::setReferenceFrame(const OpenSim::RigidFrame& aFrame)
{
    updConnector<RigidFrame>("reference_frame").connect(aFrame);
}

SimTK::Vec3 Station::reexpressLocationInFrame(const SimTK::State& s,
        OpenSim::Frame& aFrame) const
{
	// Get the transform from the station's frame to the other frame
	SimTK::Vec3 currentLocation = get_location();
    return getReferenceFrame().expressPointInAnotherFrame(s, currentLocation,
            aFrame);
}
