/* -------------------------------------------------------------------------- *
 *                    OpenSim:  SpringGeneralizedForce.cpp                    *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
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
#include <OpenSim/Simulation/SimbodyEngine/Coordinate.h>
#include "SpringGeneralizedForce.h"

using namespace OpenSim;
using namespace std;


//==============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//==============================================================================

//_____________________________________________________________________________
// This also serves as the default constructor.
SpringGeneralizedForce::SpringGeneralizedForce(const string& coordinateName)
{
    setNull();
    constructProperties();

    if (!coordinateName.empty())
        set_coordinate(coordinateName);
}

//_____________________________________________________________________________
// Set the data members of this force to their null values.
void SpringGeneralizedForce::setNull()
{
    setAuthors("Frank C. Anderson ");
}

    
//_____________________________________________________________________________
//
/**
 * Set the data members of this force to their null values.
 */
void SpringGeneralizedForce::constructProperties()
{
    constructProperty_coordinate();
    constructProperty_stiffness(0.0);
    constructProperty_rest_length(0.0);
    constructProperty_viscosity(0.0);
}

//_____________________________________________________________________________
/**
 * Sets the _model pointer to proper value
 * _coordinate is actually set inside _createSystem
 */
void SpringGeneralizedForce::extendConnectToModel(Model& model)
{
    Super::extendConnectToModel(model);

    _coord = &model.updCoordinateSet().get(get_coordinate());
}

//==============================================================================
// GET AND SET
//==============================================================================
//-----------------------------------------------------------------------------
// REST LENGTH
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the rest length of the spring.
 *
 * @param aRestLength Rest length of the spring.
 */
void SpringGeneralizedForce::
setRestLength(double aRestLength)
{
    set_rest_length(aRestLength);
}
//_____________________________________________________________________________
/**
 * Get the rest length of the spring.
 *
 * @return Rest length of the spring.
 */
double SpringGeneralizedForce::
getRestLength() const
{
    return get_rest_length();
}

//-----------------------------------------------------------------------------
// VISCOSITY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the viscosity of the spring.  Normally the viscosity should be a
 * positive number.  Negative viscosities will put energy into the system
 * rather than apply a damping force.
 *
 * @param aViscosity Viscosity of the spring.
 */
void SpringGeneralizedForce::
setViscosity(double aViscosity)
{
    set_viscosity(aViscosity);
}
//_____________________________________________________________________________
/**
 * Get the viscosity of the spring.
 *
 * @return Stiffness of the spring.
 */
double SpringGeneralizedForce::
getViscosity() const
{
    return get_viscosity();
}

//-----------------------------------------------------------------------------
// STIFFNESS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the stiffness of the spring.  Normally the stiffness is a positive
 * quantity.  Negative stiffnesses will result in an unstable system- the
 * force will push away from the rest length instead of pulling toward it.
 *
 * @param aStiffness Stiffness of the spring force.
 */
void SpringGeneralizedForce::
setStiffness(double aStiffness)
{
    set_stiffness(aStiffness);
}
//_____________________________________________________________________________
/**
 * Get the stiffness of the force.
 *
 * @return Stiffness of the force.
 */
double SpringGeneralizedForce::
getStiffness() const
{

    return get_stiffness();
}


//==============================================================================
// COMPUTATIONS
//==============================================================================
//_____________________________________________________________________________
/**
 * Compute all quantities necessary for applying the spring force to the
 * model.
 * Force applied = -stiffness * (_coordinateValue - restLength) 
 *                   - viscosity * _coordinateSpeed
 */
void SpringGeneralizedForce::computeForce(const SimTK::State& s, 
                                  SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
                                  SimTK::Vector& generalizedForces) const
{
    if( !_model || !_coord ) return;

    // FORCE
    applyGeneralizedForce(s, *_coord, computeForceMagnitude(s), generalizedForces);
}
//_____________________________________________________________________________
/**
 * Sets the actual Coordinate reference _coord
 */
void SpringGeneralizedForce::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);

    if (_model) {
        SpringGeneralizedForce* mthis = 
            const_cast<SpringGeneralizedForce*>(this);
        mthis->_coord = &_model->updCoordinateSet().get(get_coordinate());
    }
}
/** 
 * Methods to query a Force for the value actually applied during simulation
 * The names of the quantities (column labels) is returned by this first function
 * getRecordLabels()
 */
OpenSim::Array<std::string> SpringGeneralizedForce::getRecordLabels() const {
    OpenSim::Array<std::string> labels("");
    labels.append(getName()+"_Force");
    return labels;
}
/**
 * Given SimTK::State object extract all the values necessary to report forces, application location
 * frame, etc. used in conjunction with getRecordLabels and should return same size Array
 */
OpenSim::Array<double> SpringGeneralizedForce::getRecordValues(const SimTK::State& state) const {
    OpenSim::Array<double> values(1);

    values.append(computeForceMagnitude(state));
    return values;
};

/**
 * Given SimTK::State object Compute the (signed) magnitude of the force applied
 * along the _coordinate
 */
double SpringGeneralizedForce::
computeForceMagnitude(const SimTK::State& s) const
{
    double q = _coord->getValue(s);
    double speed =  _coord->getSpeedValue(s);
    double force = -getStiffness()*(q - get_rest_length()) 
                        - get_viscosity()*speed;
    return force;
}
