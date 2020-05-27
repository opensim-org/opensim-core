/* -------------------------------------------------------------------------- *
 *                      OpenSim:  CoordinateActuator.cpp                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
 * Contributor(s): Frank C. Anderson                                          *
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

//==============================================================================
// INCLUDES
//==============================================================================
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <OpenSim/Simulation/Model/ForceSet.h>

#include "CoordinateActuator.h"

using namespace OpenSim;
using namespace std;

//==============================================================================
// CONSTRUCTOR
//==============================================================================
// Uses default (compiler-generated) destructor, copy constructor, copy 
// assignment operator.

//_____________________________________________________________________________
// Also serves as default constructor (with coordinateName="").
CoordinateActuator::CoordinateActuator(const string& coordinateName)
{
    setNull();
    constructProperties();

    if (!coordinateName.empty())
        set_coordinate(coordinateName);
}
//_____________________________________________________________________________
// Set the data members of this actuator to their null values.
void CoordinateActuator::setNull()
{
    setAuthors("Ajay Seth");
}

//_____________________________________________________________________________
// Allocate and initialize properties.
void CoordinateActuator::constructProperties()
{
    constructProperty_coordinate();
    constructProperty_optimal_force(1.0);
}


//==============================================================================
// GET AND SET
//==============================================================================
//-----------------------------------------------------------------------------
// CoordinateID
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the generalized coordinate to which the coordinate actuator is applied.
 *
 * @param aCoordinate Pointer to the generalized coordinate.
 */
void CoordinateActuator::setCoordinate(Coordinate* coordinate)
{
    _coord = coordinate;
    if (coordinate)
        set_coordinate(coordinate->getName());
}
//_____________________________________________________________________________
/**
 * Get the generalized coordinate to which the coordinate actuator
 * is applied.
 *
 * @return Pointer to the coordinate
 */
Coordinate* CoordinateActuator::getCoordinate() const
{
    return _coord.get();
}

//-----------------------------------------------------------------------------
// OPTIMAL FORCE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the optimal force of the force.
 *
 * @param aOptimalForce Optimal force.
 */
void CoordinateActuator::setOptimalForce(double optimalForce)
{
    set_optimal_force(optimalForce);
}
//_____________________________________________________________________________
/**
 * Get the optimal force of the force.
 *
 * @return Optimal force.
 */
double CoordinateActuator::getOptimalForce() const
{
    return get_optimal_force();
}
//_____________________________________________________________________________
/**
 * Get the stress of the force. This would be the force or torque provided by 
 * this actuator divided by its optimal force.
 * @return Stress.
 */
double CoordinateActuator::getStress( const SimTK::State& s) const
{
    return std::abs(getActuation(s) / getOptimalForce()); 
}


//==============================================================================
// COMPUTATIONS
//==============================================================================
//_____________________________________________________________________________
/**
 * Compute all quantities necessary for applying the actuator force to the
 * model.
 */
double CoordinateActuator::computeActuation( const SimTK::State& s ) const
{
    if(!_model)
        return 0.0;

    // FORCE
    return getControl(s) * getOptimalForce();
}


//==============================================================================
// UTILITY
//==============================================================================
//_____________________________________________________________________________
/**
 */
ForceSet *CoordinateActuator::
CreateForceSetOfCoordinateActuatorsForModel(const SimTK::State& s, Model& aModel,double aOptimalForce,bool aIncludeLockedAndConstrainedCoordinates)
{
    ForceSet& as = aModel.updForceSet();
    as.setSize(0);
    auto coordinates = aModel.getCoordinatesInMultibodyTreeOrder();
    for(size_t i=0u; i < coordinates.size(); ++i) {
        const Coordinate& coord = *coordinates[i];
        if(!aIncludeLockedAndConstrainedCoordinates && (coord.isConstrained(s))) continue;
        CoordinateActuator *actuator = new CoordinateActuator();
        actuator->setCoordinate(const_cast<Coordinate*>(&coord));
        actuator->setName(coord.getName()+"_actuator");
        actuator->setOptimalForce(aOptimalForce);
        as.append(actuator);
    }

    aModel.invalidateSystem();
    return &as;
}

//==============================================================================
// APPLICATION
//==============================================================================
//_____________________________________________________________________________
/**
 * Apply the actuator force to BodyA and BodyB.
 */
void CoordinateActuator::computeForce( const SimTK::State& s, 
                               SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
                               SimTK::Vector& mobilityForces) const
{
    if(!_model) return;

   double force;
   if (isActuationOverridden(s)) {
       force = computeOverrideActuation(s);
    } else {
       force = computeActuation(s);
    }
   setActuation(s, force);

    if(isCoordinateValid()){
        applyGeneralizedForce(s, *_coord, getActuation(s), mobilityForces);
    } else {
        log_warn("CoordinateActuator::computeForce: Invalid coordinate");
    }
}

double CoordinateActuator::
getSpeed( const SimTK::State& s) const
{
    assert(_coord);
    return _coord->getSpeedValue(s);
};

//_____________________________________________________________________________
/**
 * Perform some setup functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel OpenSim model containing this CoordinateActuator.
 */
void CoordinateActuator::extendConnectToModel(Model& aModel)
{
    Super::extendConnectToModel(aModel);

    string errorMessage;

    // This will fail if no coordinate has been specified.
    const string& coordName = get_coordinate();

    // Look up the coordinate
    if (!_model->updCoordinateSet().contains(coordName)) {
        errorMessage = "CoordinateActuator: Invalid coordinate (" + coordName + ") specified in Actuator " + getName();
        throw (Exception(errorMessage.c_str()));
    }
    else
        _coord = &_model->updCoordinateSet().get(coordName);
}


//_____________________________________________________________________________
// Is the coordinate valid?
bool CoordinateActuator::isCoordinateValid() const
{
    if ( !_model || !_coord )
        return false;

    return true;
}


