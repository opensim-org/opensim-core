/* -------------------------------------------------------------------------- *
 *                          OpenSim:  Reporter.cpp                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2016 Stanford University and the Authors                *
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

#include "Reporter.h"

#include <OpenSim/Common/TimeSeriesTable.h>


using namespace SimTK;

namespace OpenSim {

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
// Default constructor.
Reporter::Reporter()
{
    setNull();
    constructProperties();
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================

//_____________________________________________________________________________
// Set the data members of this Reporter to their null values.
void Reporter::setNull()
{
    setAuthors("Ajay Seth");
}

//_____________________________________________________________________________
// Define properties.
void Reporter::constructProperties()
{
    constructProperty_isDisabled(false);
}

// Create an underlying SimTK::Reporter to represent the OpenSim::Reporter in the 
// computational system.  Create a SimTK::Reporter::Custom by default.
void Reporter::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);

    ForceAdapter* adapter = new ForceAdapter(*this);
    SimTK::Reporter::Custom force(_model->updForceSubsystem(), adapter);

     // Beyond the const Component get the index so we can access the SimTK::Reporter later
    Reporter* mutableThis = const_cast<Reporter *>(this);
    mutableThis->_index = force.getForceIndex();
}


void Reporter::extendInitStateFromProperties(SimTK::State& s) const
{
    Super::extendInitStateFromProperties(s);

    SimTK::Reporter& simForce = _model->updForceSubsystem().updForce(_index);

    // Otherwise we have to change the status of the constraint
    if(get_isDisabled())
        simForce.disable(s);
    else
        simForce.enable(s);

}

void Reporter::extendSetPropertiesFromState(const SimTK::State& state)
{
    Super::extendSetPropertiesFromState(state);

    set_isDisabled(isDisabled(state));
}


//_____________________________________________________________________________
/**
 * Set whether or not this Reporter is disabled.
 * Simbody multibody system instance is realized every time the isDisabled
 * changes, BUT multiple sets to the same value have no cost.
 *
 * @param isDisabled If true the force is disabled; if false the Reporter is enabled.
 */
void Reporter::setDisabled(SimTK::State& s, bool isDisabled) const
{
    if(_index.isValid()){
        SimTK::Reporter& simtkForce = _model->updForceSubsystem().updForce(_index);
        if(isDisabled)
            simtkForce.disable(s);
        else
            simtkForce.enable(s);
    }
}

bool Reporter::isDisabled(const SimTK::State& s) const
{
    if(_index.isValid()){
        SimTK::Reporter& simtkForce = _model->updForceSubsystem().updForce(_index);
        return simtkForce.isDisabled(s);
    }
    return get_isDisabled();
}

//-----------------------------------------------------------------------------
// ABSTRACT METHODS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
double Reporter::computePotentialEnergy(const SimTK::State& state) const
{
    return 0.0;
}

//-----------------------------------------------------------------------------
// METHODS TO APPLY FORCES AND TORQUES
//-----------------------------------------------------------------------------
void Reporter::applyForceToPoint(const SimTK::State &s, const PhysicalFrame &aBody, const Vec3& aPoint, 
                                    const Vec3& aForce, Vector_<SpatialVec> &bodyForces) const
{
    _model->getMatterSubsystem().addInStationForce(s, aBody.getMobilizedBodyIndex(),
                                                   aPoint, aForce, bodyForces);
}

void Reporter::applyTorque(const SimTK::State &s, const PhysicalFrame& aBody, 
                        const Vec3& aTorque, Vector_<SpatialVec> &bodyForces) const
{
    _model->getMatterSubsystem().addInBodyTorque(s, aBody.getMobilizedBodyIndex(),
                                                 aTorque, bodyForces);
}

void Reporter::applyGeneralizedForce(const SimTK::State &s, const Coordinate &aCoord, 
                                        double aForce, Vector &mobilityForces) const
{
    _model->getMatterSubsystem().addInMobilityForce(s, SimTK::MobilizedBodyIndex(aCoord.getBodyIndex()), 
                                    SimTK::MobilizerUIndex(aCoord.getMobilizerQIndex()), aForce, mobilityForces);
}


} // end of namespace OpenSim
