// MultiplierFunction.cpp
// Author: Peter Loan
/*
 * Copyright (c)  2009, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


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
//_____________________________________________________________________________
/**
 * Copy this object.
 *
 * @return Pointer to a copy of this object.
 */
Object* MultiplierFunction::copy() const
{
	MultiplierFunction *function = new MultiplierFunction(*this);
	return function;
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
	setType("MultiplierFunction");
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

//-----------------------------------------------------------------------------
// UPDATE FROM XML NODE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Update this object based on its XML node.
 */
void MultiplierFunction::updateFromXMLNode()
{
	Function::updateFromXMLNode();
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
	if (aFunction->isA("MultiplierFunction")) {
		MultiplierFunction* mf = (MultiplierFunction*)aFunction;
		setFunction(mf->getFunction());
		setScale(mf->getScale());
	} else {
		setFunction(aFunction);
		setScale(1.0);
	}
}
