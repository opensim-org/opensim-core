/* -------------------------------------------------------------------------- *
 *                           OpenSim:  Constant.cpp                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "Constant.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
Constant::~Constant()
{
}

//_____________________________________________________________________________
/**
 * Default constructor.
 */
Constant::Constant() :
    _value(_valueProp.getValueDbl())
{
    setNull();
    setupProperties();
}

//_____________________________________________________________________________
/**
 */
Constant::Constant(double value) :
    _value(_valueProp.getValueDbl())
{
    setNull();
    setupProperties();
    setValue(value);
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 * All data members of the specified Constant are copied.
 *
 * @param aConstant Constant object to be copied.
 */
Constant::Constant(const Constant &aConstant) :
    Function(aConstant),
    _value(_valueProp.getValueDbl())
{
    setNull();
    setupProperties();
    copyData(aConstant);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set all member variables to their NULL or default values.
 */
void Constant::setNull()
{
    setAuthors("Peter Loan, Ajay Seth");
}

//_____________________________________________________________________________
/**
 * Set up the serialized member variables.  This involves both generating
 * the properties and connecting them to the local pointers used to access
 * the serialized member variables.
 */
void Constant::setupProperties()
{
    _valueProp.setName("value");
    _valueProp.setValue(0);
    _propertySet.append(&_valueProp);
}

//_____________________________________________________________________________
/**
 * Set all member variables equal to the members of another object.
 * Note that this method is private.  It is only meant for copying the data
 * members defined in this class.  It does not, for example, make any changes
 * to data members of base classes.
 *
 * @param aConstant Constant to be copied.
 */
void Constant::copyData(const Constant &aConstant)
{
    _value = aConstant._value;
    resetFunction();
}


void Constant::setValue(double aValue)
{
    _value = aValue;
    resetFunction();
}


//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 * Note that data members of the base class are also assigned.
 *
 * @param aConstant Constant to be copied.
 * @return Reference to this object.
 */
Constant& Constant::operator=(const Constant &aConstant)
{
    // BASE CLASS
    Function::operator=(aConstant);

    // DATA
    copyData(aConstant);

    return(*this);
}

//=============================================================================
// UTILITY
//=============================================================================
SimTK::Function* Constant::createSimTKFunction() const {
    return new SimTK::Function::Constant(_value, 0);
}
