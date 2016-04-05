/* -------------------------------------------------------------------------- *
 *                             OpenSim:  Point.cpp                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2015 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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
#include "Point.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Mat33;
using SimTK::Vec3;

//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
Point::Point() : ModelComponent()
{
    setAuthors("Ajay Seth");
}


void Point::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);
    SimTK::Vec3 v(SimTK::NaN);
    // If the properties, topology or coordinate values change, 
    // Stage::Position will be invalid.
    addCacheVariable("location", v, SimTK::Stage::Position);
}

const SimTK::Vec3& Point::getLocationInGround(const SimTK::State& s) const
{
    if (!getSystem().getDefaultSubsystem().
        isCacheValueRealized(s, _groundLocationIndex)){
        //cache is not valid so calculate the transform
        SimTK::Value<SimTK::Vec3>::downcast(
            getSystem().getDefaultSubsystem().
            updCacheEntry(s, _groundLocationIndex)).upd()
                = calcLocationInGround(s);
        // mark cache as up-to-date
        getSystem().getDefaultSubsystem().
            markCacheValueRealized(s, _groundLocationIndex);
    }
    return SimTK::Value<SimTK::Vec3>::downcast(
        getSystem().getDefaultSubsystem().
        getCacheEntry(s, _groundLocationIndex)).get();
}

//=============================================================================
// POINT COMPUTATIONS
//=============================================================================


//=============================================================================
// Component level realizations
//=============================================================================
void Point::extendRealizeTopology(SimTK::State& s) const
{
    Super::extendRealizeTopology(s);
    _groundLocationIndex = getCacheVariableIndex("ground_location");
}
