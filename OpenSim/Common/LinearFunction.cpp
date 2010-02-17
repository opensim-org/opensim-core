// LinearFunction.cpp
// Author: Ajay Seth
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

//_____________________________________________________________________________
/**
 * Copy this object.
 *
 * @return Pointer to a copy of this object.
 */
Object* LinearFunction::copy() const
{
	LinearFunction *aLinearFunction = new LinearFunction(*this);
	return(aLinearFunction);
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
	setType("LinearFunction");
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
