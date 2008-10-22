// SimmTranslationDof.cpp
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
#include "SimmTranslationDof.h"
#include <OpenSim/Simulation/Model/AbstractCoordinate.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SimmTranslationDof::SimmTranslationDof()
{
	setNull();

	_axis[0] = _axis[1] = _axis[2] = 0.0;
	_axisIndex = xTranslation;
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SimmTranslationDof::~SimmTranslationDof()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aDof SimmTranslationDof to be copied.
 */
SimmTranslationDof::SimmTranslationDof(const SimmTranslationDof &aDof) :
   AbstractDof01_05(aDof)
{
	setNull();
	copyData(aDof);
}

//_____________________________________________________________________________
/**
 * Copy this dof and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this SimmTranslationDof.
 */
Object* SimmTranslationDof::copy() const
{
	SimmTranslationDof *dof = new SimmTranslationDof(*this);
	return(dof);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Perform some set up functions that need to be done each time the
 * SimmTranslationDof is deserialized.
 */
void SimmTranslationDof::updateFromXMLNode()
{
	AbstractDof01_05::updateFromXMLNode();

	if (_name == TX_NAME)
	{
		_axis[0] = 1.0;
		_axisIndex = xTranslation;
	}
	else if (_name == TY_NAME)
	{
		_axis[1] = 1.0;
		_axisIndex = yTranslation;
	}
	else if (_name == TZ_NAME)
	{
		_axis[2] = 1.0;
		_axisIndex = zTranslation;
	}
}

//_____________________________________________________________________________
/**
 * Copy data members from one SimmTranslationDof to another.
 *
 * @param aDof SimmTranslationDof to be copied.
 */
void SimmTranslationDof::copyData(const SimmTranslationDof &aDof)
{
	aDof.getAxis(_axis);
	_axisIndex = aDof._axisIndex;
}

//_____________________________________________________________________________
/**
 * Set the data members of this SimmTranslationDof to their null values.
 */
void SimmTranslationDof::setNull()
{
	setType("SimmTranslationDof");
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
SimmTranslationDof& SimmTranslationDof::operator=(const SimmTranslationDof &aDof)
{
	// BASE CLASS
	AbstractDof01_05::operator=(aDof);

	copyData(aDof);

	return(*this);
}

//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Get the current value of the translation dof
 *
 * @return The current value of the dof.
 */
double SimmTranslationDof::getValue()
{
	if (_coordinate)
		return _function->evaluate(0, _coordinate->getValue(), 0.0, 0.0);
	else
		return _function->evaluate(0, 0.0, 0.0, 0.0);
}

//_____________________________________________________________________________
/**
 * Set the translation axis.
 *
 * @param aAxis Translation axis.
 */
void SimmTranslationDof::setAxis(const SimTK::Vec3& aAxis)
{
	_axis = aAxis;
}
//_____________________________________________________________________________
/**
 * Get the translation axis.
 *
 * @param rAxis the translation axis is returned here.
 */
void SimmTranslationDof::getAxis(SimTK::Vec3& rAxis) const
{
	rAxis = _axis;
}

//_____________________________________________________________________________
/**
 * Get the translation.
 *
 * @param rVec the translation is returned here.
 */
void SimmTranslationDof::getTranslation(double rVec[4])
{
	double value = getValue();

	rVec[0] = _axis[0] * value;
	rVec[1] = _axis[1] * value;
	rVec[2] = _axis[2] * value;
	rVec[3] = 1.0;
}
