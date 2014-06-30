/* -------------------------------------------------------------------------- *
 *                            OpenSim:  Station.cpp                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ayman Habib                                                      *
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


void Station::constructStructuralConnectors()
{
	//constructStructuralConnector<Frame>("parent_body");
}


//_____________________________________________________________________________
/**
 * Change the body that this Station is attached to. It assumes that the body is
 * already set, so that connectStationToModel() needs to be called to update 
 * dependent information.
 *
 * @param aBody Reference to the body.
 *
void Station::changeBody( OpenSim::Body& aBody)
{

	setBodyName(aBody.getName());
}
*/
//_____________________________________________________________________________
/**
 * Change the body that this Station is attached to. It assumes that the body is
 * already set, so that connectStationToModel() needs to be called to update 
 * dependent information.
 *
 * @param s State.
 * @param aBody Reference to the body.
 *
void Station::changeBodyPreserveLocation(const SimTK::State& s, OpenSim::Body& aBody)
{

	// Preserve location means to switch bodies without changing
	// the location of the Station in the inertial reference frame.
    //aBody.getModel().getSimbodyEngine().transformPosition(s, *_body, _offset, aBody, _offset);

	setBodyName(aBody.getName());
}
//_____________________________________________________________________________
/**
* Perform some set up functions that happen after the
* object has been deserialized or copied.
*
* @param aModel OpenSim model containing this Station.
*/
void Station::connectToModel(Model& aModel)
{
	Super::connectToModel(aModel);

}
