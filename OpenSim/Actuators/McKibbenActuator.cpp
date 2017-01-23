/* -------------------------------------------------------------------------- *
 *                       OpenSim:  McKibbenActuator.cpp                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Nabeel Allana                                                   *
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
#include "McKibbenActuator.h"

using namespace OpenSim;
using std::string;
using SimTK::Vec3; using SimTK::Vector_; using SimTK::Vector; 
using SimTK::SpatialVec; using SimTK::UnitVec3; using SimTK::State;


//==============================================================================
// CONSTRUCTOR(S)
//==============================================================================
// default destructor and copy construction & assignment
//_____________________________________________________________________________
// Default constructor.
McKibbenActuator::McKibbenActuator()
{
    constructProperties();
}
//_____________________________________________________________________________
// Constructor with given body names.
McKibbenActuator::McKibbenActuator(const string& name, double num_turns, double thread_length)
{
    constructProperties();
    setName(name);
    set_number_of_turns(num_turns);
    set_thread_length(thread_length);
}

//_____________________________________________________________________________
// Construct and initialize properties.
void McKibbenActuator::constructProperties()
{
    setAuthors("Nabeel Allana");
    constructProperty_number_of_turns(1.5);
    constructProperty_thread_length(200);
    constructProperty_cord_length(0);
}


//==============================================================================
// COMPUTATIONS
//==============================================================================
//_____________________________________________________________________________
//_____________________________________________________________________________
/**
 * Compute all quantities necessary for applying the actuator force to the
 * model.
 *
 * @param s current SimTK::State 
 */

double McKibbenActuator::computeActuation( const SimTK::State& s ) const
{
    if(!_model) return 0;

    double length = getLength(s) - getCordLength();
    double pressure = getControl(s);
    double B = getThreadLength();
    double N = getNumberOfTurns();
    
    
    double force = (pressure / (4*pow(N,2)*SimTK::Pi)) * (3*pow(length, 2) - pow(B,2));
    setActuation(s, force);
    
    return force;
}

//==============================================================================
// APPLICATION
//==============================================================================
//_____________________________________________________________________________
/**
 * Apply the actuator force to the path
 *
 * @param s current SimTK::State
 */
void McKibbenActuator::computeForce(const SimTK::State& s, 
                                SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
                                SimTK::Vector& generalizedForces) const
{

    double actuation = computeActuation(s);

    getGeometryPath().addInEquivalentForces(s, actuation, bodyForces, generalizedForces);
}
//_____________________________________________________________________________
/**
 * Sets the actual Body references _bodyA and _bodyB
 */
void McKibbenActuator::extendConnectToModel(Model& model)
{
    Super::extendConnectToModel(model);
}
