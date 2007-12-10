// MuscleViaPoint.cpp
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
#include "MuscleViaPoint.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/AbstractMuscle.h>
#include <OpenSim/Simulation/Model/AbstractBody.h>
#include <OpenSim/Simulation/Model/AbstractCoordinate.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <OpenSim/Simulation/Model/AbstractDynamicsEngine.h>

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
MuscleViaPoint::MuscleViaPoint() :
   _range(_rangeProp.getValueDblArray()),
	_coordinateName(_coordinateNameProp.getValueStr()),
	_coordinate(NULL)
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
MuscleViaPoint::~MuscleViaPoint()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aPoint MuscleViaPoint to be copied.
 */
MuscleViaPoint::MuscleViaPoint(const MuscleViaPoint &aPoint) :
   MusclePoint(aPoint),
   _range(_rangeProp.getValueDblArray()),
	_coordinateName(_coordinateNameProp.getValueStr()),
	_coordinate(NULL)
{
	setNull();
	setupProperties();
	copyData(aPoint);
}

//_____________________________________________________________________________
/**
 * Copy this muscle via point and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this MuscleViaPoint.
 */
Object* MuscleViaPoint::copy() const
{
	MuscleViaPoint *pt = new MuscleViaPoint(*this);
	return(pt);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one MuscleViaPoint to another.
 *
 * @param aPoint MuscleViaPoint to be copied.
 */
void MuscleViaPoint::copyData(const MuscleViaPoint &aPoint)
{
	_range = aPoint._range;
	_coordinateName = aPoint._coordinateName;
	_coordinate = aPoint._coordinate;
}

//_____________________________________________________________________________
/**
 * Set the data members of this MuscleViaPoint to their null values.
 */
void MuscleViaPoint::setNull()
{
	setType("MuscleViaPoint");
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void MuscleViaPoint::setupProperties()
{
	const double defaultRange[] = {0.0, 0.0};
	_rangeProp.setName("range");
	_rangeProp.setValue(2, defaultRange);
	_rangeProp.setAllowableArraySize(2);
	_propertySet.append(&_rangeProp);

	_coordinateNameProp.setName("coordinate");
	_propertySet.append(&_coordinateNameProp);
}

//_____________________________________________________________________________
/**
 * Set the coordinate that this point is linked to.
 *
 * @return Whether or not this point is active.
 */
void MuscleViaPoint::setCoordinate(AbstractCoordinate& aCoordinate)
{
	if (&aCoordinate != _coordinate)
	{
	   _coordinate = &aCoordinate;
	   _coordinateName = _coordinate->getName();
		_muscle->invalidatePath();
	}
}

//_____________________________________________________________________________
/**
 * Set the range min.
 *
 * @param aRange range min to change to.
 */
void MuscleViaPoint::setRangeMin(double aMin)
{
	if (aMin <= _range[1])
	{
		_range[0] = aMin;
		_muscle->invalidatePath();
	}
}

//_____________________________________________________________________________
/**
 * Set the range max.
 *
 * @param aRange range max to change to.
 */
void MuscleViaPoint::setRangeMax(double aMax)
{
	if (aMax >= _range[0])
	{
		_range[1] = aMax;
		_muscle->invalidatePath();
	}
}

//_____________________________________________________________________________
/**
 * Determine if this point is active by checking the value of the
 * coordinate that it is linked to.
 *
 * @return Whether or not this point is active.
 */
bool MuscleViaPoint::isActive() const
{
	if (_coordinate)
	{
		double value = _coordinate->getValue();
		if (value >= _range[0] - _coordinate->getTolerance() &&
			 value <= _range[1] + _coordinate->getTolerance())
			return true;
	}

	return false;
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel model containing this MuscleViaPoint.
 */
void MuscleViaPoint::setup(Model* aModel, AbstractMuscle* aMuscle)
{
	// base class
	MusclePoint::setup(aModel, aMuscle);

	/* Look up the coordinate by name in the dynamics engine and
	 * store a pointer to it.
	 */
	_coordinate = aModel->getDynamicsEngine().getCoordinateSet()->get(_coordinateName);
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
MuscleViaPoint& MuscleViaPoint::operator=(const MuscleViaPoint &aPoint)
{
	// BASE CLASS
	MusclePoint::operator=(aPoint);

	copyData(aPoint);

	return(*this);
}
