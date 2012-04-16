// Constant.cpp
// Author: Peter Loan
/*
 * Copyright (c)  2006, Stanford University. All rights reserved. 
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
#include "Constant.h"
#include "XYFunctionInterface.h"

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
