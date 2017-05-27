/* -------------------------------------------------------------------------- *
 *                        OpenSim:  LinearFunction.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
#include "LinearFunction.h"

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
LinearFunction::~LinearFunction()
{
}

//_____________________________________________________________________________
/**
 * Default constructor.
 */
LinearFunction::LinearFunction() :
    _coefficients(_coefficientsProp.getValueDblArray())
{
    setNull();
    setupProperties();
}

//_____________________________________________________________________________
/**
 * Construct by defining the slope and intercept
 */
LinearFunction::LinearFunction(double slope, double intercept) :
    _coefficients(_coefficientsProp.getValueDblArray())
{   
    setNull();
    setupProperties();
    setSlope(slope);
    setIntercept(intercept);
}
//_____________________________________________________________________________
/**
 */
LinearFunction::LinearFunction(Array<double> coefficients) :
    _coefficients(_coefficientsProp.getValueDblArray())
{
    setNull();
    setupProperties();
    setCoefficients(coefficients);
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 * All data members of the specified LinearFunction are copied.
 *
 * @param aLinearFunction LinearFunction object to be copied.
 */
LinearFunction::LinearFunction(const LinearFunction &aLinearFunction) :
    Function(aLinearFunction),
    _coefficients(_coefficientsProp.getValueDblArray())
{
    setNull();
    setupProperties();
    copyData(aLinearFunction);
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set all member variables to their NULL or default values.
 */
void LinearFunction::setNull()
{
    setAuthors("Ajay Seth");
}

//_____________________________________________________________________________
/**
 * Set up the serialized member variables.  This involves both generating
 * the properties and connecting them to the local pointers used to access
 * the serialized member variables.
 */
void LinearFunction::setupProperties()
{
    Array<double> intitCoeffs(0,2);
    intitCoeffs[0]=1;

    _coefficientsProp.setName("coefficients");
    _coefficientsProp.setValue(intitCoeffs);
    _propertySet.append(&_coefficientsProp);
}

//_____________________________________________________________________________
/**
 * Set all member variables equal to the members of another object.
 * Note that this method is private.  It is only meant for copying the data
 * members defined in this class.  It does not, for example, make any changes
 * to data members of base classes.
 *
 * @param aLinearFunction LinearFunction to be copied.
 */
void LinearFunction::copyData(const LinearFunction &aLinearFunction)
{
    _coefficients = aLinearFunction._coefficients;
    resetFunction();
}

void LinearFunction::setCoefficients(Array<double> coefficients)
{
    _coefficients = coefficients;
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
 * @param aLinearFunction LinearFunction to be copied.
 * @return Reference to this object.
 */
LinearFunction& LinearFunction::operator=(const LinearFunction &aLinearFunction)
{
    // BASE CLASS
    Function::operator=(aLinearFunction);

    // DATA
    copyData(aLinearFunction);

    return(*this);
}

//=============================================================================
// UTILITY
//=============================================================================
SimTK::Function* LinearFunction::createSimTKFunction() const 
{
    SimTK::Vector coeffs(_coefficients.getSize(), &_coefficients[0]);
    return new SimTK::Function::Linear(coeffs);
}
