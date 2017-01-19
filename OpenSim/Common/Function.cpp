/* -------------------------------------------------------------------------- *
 *                           OpenSim:  Function.cpp                           *
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


// INCLUDES
#include "Function.h"


using namespace OpenSim;
using namespace std;
using SimTK::Vector;


//=============================================================================
// STATICS
//=============================================================================


//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
Function::~Function()
{
    if (_function != NULL)
        delete _function;
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
Function::Function() :
    _function(NULL)
{
    setNull();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aFunction Function to copy.
 */
Function::Function(const Function &aFunction) :
    Object(aFunction),
    _function(NULL)
{
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set all member variables to NULL values.
 */
void Function::
setNull()
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
Function& Function::
operator=(const Function &aFunction)
{
    // BASE CLASS
    Object::operator=(aFunction);

    return(*this);
}


//=============================================================================
// UTILITIES
//=============================================================================
Function* Function::makeFunctionOfType(Function* aFunction, const string& aNewTypeName)
{
    Function* newFunction = NULL;

    if (aFunction != NULL) {
        Object* newObject = Object::newInstanceOfType(aNewTypeName);
        if (newObject) {
            newFunction = dynamic_cast<Function*>(newObject);
            if (newFunction) {
                newFunction->init(aFunction);
            }
        }
    }

    return newFunction;
}

//=============================================================================
// EVALUATE
//=============================================================================
/*
double Function::evaluate(int aDerivOrder,double aX,double aY,double aZ) const
{
    SimTK::Vector workX(getArgumentSize(), aX);
    if (aDerivOrder == 0)
        return calcValue(workX);
    std::vector<int> workDeriv(aDerivOrder);
    for (int i = 0; i < aDerivOrder; ++i)
        workDeriv[i] = 0;
    return calcDerivative(workDeriv, workX);
}
*/

/**
 * Evaluates total first derivative using the chain rule.
 *
double Function::
evaluateTotalFirstDerivative(double aX,double aDxdt)
{
    return evaluate(1,aX) * aDxdt;
}
*/
/**
 * Evaluates total second derivative using the chain rule.
 *
double Function::
evaluateTotalSecondDerivative(double aX,double aDxdt,double aD2xdt2)
{
    return evaluate(1,aX) * aD2xdt2 + evaluate(2,aX) * aDxdt * aDxdt;
}
*/
double Function::calcValue(const Vector& x) const
{
    if (_function == NULL)
        _function = createSimTKFunction();
    return _function->calcValue(x);
}

double Function::calcDerivative(const std::vector<int>& derivComponents, const Vector& x) const
{
    if (_function == NULL)
        _function = createSimTKFunction();
    return _function->calcDerivative(derivComponents, x);
}

int Function::getArgumentSize() const
{
    if (_function == NULL)
        _function = createSimTKFunction();
    return _function->getArgumentSize();
}

int Function::getMaxDerivativeOrder() const
{
    if (_function == NULL)
        _function = createSimTKFunction();
    return _function->getMaxDerivativeOrder();
}

void Function::resetFunction()
{
    if (_function != NULL)
        delete _function;
    _function = NULL;
}
