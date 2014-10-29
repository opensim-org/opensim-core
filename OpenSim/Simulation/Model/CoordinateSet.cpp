/* -------------------------------------------------------------------------- *
 *                        OpenSim:  CoordinateSet.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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

#include "CoordinateSet.h"
#include <OpenSim/Simulation/SimbodyEngine/Joint.h>
#include <OpenSim/Simulation/SimbodyEngine/Coordinate.h>
#include <OpenSim/Simulation/Model/Model.h>

using namespace std;
using namespace OpenSim;

//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
CoordinateSet::~CoordinateSet(void)
{
}

//_____________________________________________________________________________
/**
 * Default constructor of a CoordinateSet.
 */
CoordinateSet::CoordinateSet() :
    ModelComponentSet<Coordinate>()
{
    setNull();
}

//_____________________________________________________________________________
/**
 * Copy constructor of a CoordinateSet.
 */
CoordinateSet::CoordinateSet(const CoordinateSet& aCoordinateSet):
    ModelComponentSet<Coordinate>(aCoordinateSet)
{
    setNull();
    *this = aCoordinateSet;
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
/**
 * Set the data members of this CoordinateSet to their null values.
 */
void CoordinateSet::setNull()
{
}
    
/**
  * Populate this flat list of Coordinates given a Model that has been set up
  */
void CoordinateSet::populate(Model& model)
{
    setModel(model);
    // Append Coordinate from Joint's cordrinate set to the model's set as pointers
    setMemoryOwner(false);
    setSize(0);

    for(int i=0; i< model.getJointSet().getSize(); i++){
        Joint& nextJoint = model.getJointSet().get(i);
        for(int j=0; j< nextJoint.numCoordinates(); j++){
            // Append a pointer (address) otherwise the model will get a copy that will not be updated properly
            adoptAndAppend(&(nextJoint.getCoordinateSet()[j]));
        }
    }
}


//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
#ifndef SWIG
CoordinateSet& CoordinateSet::operator=(const CoordinateSet &aCoordinateSet)
{
    Set<Coordinate>::operator=(aCoordinateSet);
    return (*this);
}
#endif
