/* -------------------------------------------------------------------------- *
 *                      OpenSim:  MultiplierFunction.cpp                      *
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


// C++ INCLUDES
#include "MultiplierFunction.h"
#include "FunctionAdapter.h"

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
MultiplierFunction::~MultiplierFunction()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
MultiplierFunction::MultiplierFunction() :
   _osFunction(_osFunctionProp.getValueObjPtrRef()),
   _scale(_scaleProp.getValueDbl())
{
    setNull();
}
//_____________________________________________________________________________
/**
 */
MultiplierFunction::MultiplierFunction(Function* aFunction) :
   _osFunction(_osFunctionProp.getValueObjPtrRef()),
   _scale(_scaleProp.getValueDbl())
{
    setNull();
    setFunction(aFunction);
}
//_____________________________________________________________________________
/**
 */
MultiplierFunction::MultiplierFunction(Function* aFunction, double aScaleFactor) :
   _osFunction(_osFunctionProp.getValueObjPtrRef()),
   _scale(_scaleProp.getValueDbl())
{
    setNull();
    setFunction(aFunction);
    setScale(aScaleFactor);
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 * All data members of the specified function are copied.
 *
 * @param aFunction MultiplierFunction object to be copied.
 */
MultiplierFunction::MultiplierFunction(const MultiplierFunction &aFunction) :
    Function(aFunction),
   _osFunction(_osFunctionProp.getValueObjPtrRef()),
   _scale(_scaleProp.getValueDbl())
{
    setEqual(aFunction);
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set all member variables to their NULL or default values.
 */
void MultiplierFunction::setNull()
{
    setAuthors("Peter Loan");
    setupProperties();
}
//_____________________________________________________________________________
/**
 * Set up the serialized member variables.  This involves both generating
 * the properties and connecting them to the local pointers used to access
 * the serialized member variables.
 */
void MultiplierFunction::setupProperties()
{
    _osFunctionProp.setName("function");
    _propertySet.append(&_osFunctionProp);

    _scaleProp.setName("scale");
    _scaleProp.setValue(1.0);
    _propertySet.append(&_scaleProp);
}

void MultiplierFunction::setEqual(const MultiplierFunction& aFunction)
{
    setNull();
    setFunction((Function*)(Object::SafeCopy(aFunction.getFunction())));
    setScale(aFunction.getScale());
}

//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 * Note that data members of the base class are also assigned.
 *
 * @return Reference to this object.
 */
MultiplierFunction& MultiplierFunction::operator=(const MultiplierFunction &aFunction)
{
    // BASE CLASS
    Function::operator=(aFunction);

    // DATA
    setEqual(aFunction);

    return *this;
}

//--------------------------------------------------------------------------
// SET AND GET
//--------------------------------------------------------------------------
void MultiplierFunction::setFunction(Function* aFunction)
{
    _osFunction = aFunction;
}

void MultiplierFunction::setScale(double aScaleFactor)
{
    _scale = aScaleFactor;
}

double MultiplierFunction::calcDerivative(const std::vector<int>& derivComponents, const Vector& x) const
{
    if (_osFunction)
        return _osFunction->calcDerivative(derivComponents, x) * _scale;
    else {
        throw Exception("MultiplierFunction::calcDerivative(): _osFunction is NULL.");
        return 0.0;
    }
}

double MultiplierFunction::calcValue(const SimTK::Vector& x) const
{
    if (_osFunction)
        return _osFunction->calcValue(x) * _scale;
    else {
        throw Exception("MultiplierFunction::calcValue(): _osFunction is NULL.");
        return 0.0;
    }
}

int MultiplierFunction::getArgumentSize() const
{
    if (_osFunction)
        return _osFunction->getArgumentSize();
    else {
        throw Exception("MultiplierFunction::getArgumentSize(): _osFunction is NULL.");
        return 0;
    }
}

int MultiplierFunction::getMaxDerivativeOrder() const
{
    if (_osFunction)
        return _osFunction->getMaxDerivativeOrder();
    else {
        throw Exception("MultiplierFunction::getMaxDerivativeOrder(): _osFunction is NULL.");
        return 0;
    }
}

SimTK::Function* MultiplierFunction::createSimTKFunction() const {
    return new FunctionAdapter(*this);
}

void MultiplierFunction::init(Function* aFunction)
{
    if (aFunction->getConcreteClassName()==("MultiplierFunction")) {
        MultiplierFunction* mf = (MultiplierFunction*)aFunction;
        setFunction(mf->getFunction());
        setScale(mf->getScale());
    } else {
        setFunction(aFunction);
        setScale(1.0);
    }
}
