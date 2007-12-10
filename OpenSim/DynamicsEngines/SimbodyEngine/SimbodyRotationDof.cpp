// SimbodyRotationDof.cpp
// Author: Peter Loan, Frank C. Anderson
/*
 * Copyright (c)  2006-2007, Stanford University. All rights reserved. 
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
#include "SimbodyRotationDof.h"
#include <OpenSim/Simulation/Model/AbstractCoordinate.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace SimTK;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SimbodyRotationDof::SimbodyRotationDof() :
   _axis(_axisProp.getValueDblArray())
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SimbodyRotationDof::~SimbodyRotationDof()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aDof SimbodyRotationDof to be copied.
 */
SimbodyRotationDof::SimbodyRotationDof(const SimbodyRotationDof &aDof) :
   AbstractDof(aDof),
   _axis(_axisProp.getValueDblArray())
{
	setNull();
	setupProperties();
	copyData(aDof);
}

//_____________________________________________________________________________
/**
 * Copy this dof and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this SimbodyRotationDof.
 */
Object* SimbodyRotationDof::copy() const
{
	SimbodyRotationDof *dof = new SimbodyRotationDof(*this);
	return(dof);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one SimbodyRotationDof to another.
 *
 * @param aDof SimbodyRotationDof to be copied.
 */
void SimbodyRotationDof::copyData(const SimbodyRotationDof &aDof)
{
	_axis = aDof._axis;
}

//_____________________________________________________________________________
/**
 * Set the data members of this SimbodyRotationDof to their null values.
 */
void SimbodyRotationDof::setNull()
{
	setType("SimbodyRotationDof");
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void SimbodyRotationDof::setupProperties()
{
	const double defaultAxis[] = {1.0, 0.0, 0.0};
	_axisProp.setName("axis");
	_axisProp.setValue(3, defaultAxis);
	_propertySet.append(&_axisProp);
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
SimbodyRotationDof& SimbodyRotationDof::operator=(const SimbodyRotationDof &aDof)
{
	// BASE CLASS
	AbstractDof::operator=(aDof);

	copyData(aDof);

	return(*this);
}

//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the rotation axis.
 *
 * @param aAxis Rotation axis.
 */
void SimbodyRotationDof::
setAxis(const double aAxis[3])
{
	_axis[0] = aAxis[0];
	_axis[1] = aAxis[1];
	_axis[2] = aAxis[2];
}
//_____________________________________________________________________________
/**
 * Get the rotation axis.
 *
 * @param rAxis the rotation axis is returned here.
 */
void SimbodyRotationDof::
getAxis(double rAxis[3]) const
{
	rAxis[0] = _axis[0];
	rAxis[1] = _axis[1];
	rAxis[2] = _axis[2];
}

//_____________________________________________________________________________
/**
 * Get the current value of the rotation dof
 *
 * @return The current value of the dof.
 */
double SimbodyRotationDof::getValue()
{
	if (_coordinate)
		return _function->evaluate(0, _coordinate->getValue(), 0.0, 0.0);
	else
		return _function->evaluate(0, 0.0, 0.0, 0.0);
}

void SimbodyRotationDof::peteTest()
{
	cout << "RotationDof: " << getName() << endl;
	cout << "   value: " << getValue() << endl;
	cout << "   coordinate: " << _coordinateName << endl;
	if (_function) cout << "   function: " << *_function << endl;
}
