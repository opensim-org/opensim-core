/* -------------------------------------------------------------------------- *
 *                  OpenSim:  VectorFunctionUncoupledNxN.cpp                  *
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

/*  
 * Author: Frank C. Anderson 
 */


// INCLUDES
#include "VectorFunctionUncoupledNxN.h"
#include "PropertyDbl.h"




using namespace OpenSim;
using namespace std;


//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
VectorFunctionUncoupledNxN::~VectorFunctionUncoupledNxN()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
VectorFunctionUncoupledNxN::
VectorFunctionUncoupledNxN() :
    VectorFunction(0,0)
{
    setNull();
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
VectorFunctionUncoupledNxN::
VectorFunctionUncoupledNxN(int aN) :
    VectorFunction(aN,aN)
{
    setNull();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aVectorFunction Function to copy.
 */
VectorFunctionUncoupledNxN::
VectorFunctionUncoupledNxN(const VectorFunctionUncoupledNxN &aVectorFunction) :
    VectorFunction(aVectorFunction)
{
    setNull();

    // ASSIGN
    setEqual(aVectorFunction);
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set all member variables to NULL values.
 */
void VectorFunctionUncoupledNxN::
setNull()
{
}

//_____________________________________________________________________________
/**
 * Set all member variables equal to the members of another object.
 * Note that this method is private.  It is only meant for copying the data
 * members defined in this class.  It does not, for example, make any changes
 * to data members of base classes.
 */
void VectorFunctionUncoupledNxN::
setEqual(const VectorFunctionUncoupledNxN &aVectorFunction)
{
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
VectorFunctionUncoupledNxN& VectorFunctionUncoupledNxN::
operator=(const VectorFunctionUncoupledNxN &aVectorFunction)
{
    // BASE CLASS
    VectorFunction::operator=(aVectorFunction);

    // DATA
    setEqual(aVectorFunction);

    return(*this);
}


//=============================================================================
// SET AND GET
//=============================================================================

