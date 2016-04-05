/* -------------------------------------------------------------------------- *
 *                            OpenSim:  Station.cpp                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2016 Stanford University and the Authors                *
 * Author(s): Ayman Habib, Ajay Seth                                          *
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
Station::Station() : Super()
{
    setNull();
    constructInfrastructure();
}

Station::Station(const PhysicalFrame& frame, const SimTK::Vec3& location)
    : Super()
{
    setNull();
    constructInfrastructure();
    setReferenceFrame(frame);
    set_location(location);
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
    constructConnector<PhysicalFrame>("reference_frame");
}

/*
 * Return the reference frame with respect to which this station is defined
*/
const PhysicalFrame& Station::getReferenceFrame() const
{
    return getConnector<PhysicalFrame>("reference_frame").getConnectee();
}

/*
 * setReferenceFrame sets the "reference_frame" connection
 */
void Station::setReferenceFrame(const OpenSim::PhysicalFrame& aFrame)
{
    updConnector<PhysicalFrame>("reference_frame").connect(aFrame);
}

SimTK::Vec3 Station::findLocationInFrame(const SimTK::State& s,
        const OpenSim::Frame& aFrame) const
{
    // transform location from the station's frame to the other frame
    return getReferenceFrame().findLocationInAnotherFrame(s, 
                                                get_location(), aFrame);
}

SimTK::Vec3 Station::calcLocationInGround(const SimTK::State& s) const
{
    return getReferenceFrame().getTransformInGround(s)*get_location();
}

SimTK::Vec3 Station::calcVelocityInGround(const SimTK::State& s) const
{
    const SimTK::SpatialVec& V_GF = getReferenceFrame().getVelocityInGround(s);
    return V_GF[1] + V_GF[0] % getLocationInGround(s);
}

SimTK::Vec3 Station::calcAccelerationInGround(const SimTK::State& s) const
{
    const SimTK::SpatialVec& V_GF = getReferenceFrame().getVelocityInGround(s);
    const SimTK::SpatialVec& A_GF = getReferenceFrame().getAccelerationInGround(s);
    const Vec3& r = getLocationInGround(s);
    return A_GF[1] + A_GF[0]%r +  V_GF[0] % V_GF[0] % r ;
}
