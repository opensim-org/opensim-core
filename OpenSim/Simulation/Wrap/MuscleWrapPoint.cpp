// MuscleWrapPoint.cpp
// Author: Peter Loan
/*
 * Copyright (c) 2006, Stanford University. All rights reserved. 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including 
 * without limitation the rights to use, copy, modify, merge, publish, 
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included 
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

//=============================================================================
// INCLUDES
//=============================================================================
#include "MuscleWrapPoint.h"
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
MuscleWrapPoint::MuscleWrapPoint()
{
	setNull();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
MuscleWrapPoint::~MuscleWrapPoint()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aPoint MuscleWrapPoint to be copied.
 */
MuscleWrapPoint::MuscleWrapPoint(const MuscleWrapPoint &aPoint) :
   MusclePoint(aPoint)
{
	setNull();
	copyData(aPoint);
}

//_____________________________________________________________________________
/**
 * Copy this muscle via point and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this MuscleWrapPoint.
 */
Object* MuscleWrapPoint::copy() const
{
	MuscleWrapPoint *pt = new MuscleWrapPoint(*this);
	return(pt);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one MuscleWrapPoint to another.
 *
 * @param aPoint MuscleWrapPoint to be copied.
 */
void MuscleWrapPoint::copyData(const MuscleWrapPoint &aPoint)
{
	_wrapPath = aPoint._wrapPath;
	_wrapPathLength = aPoint._wrapPathLength;
	_wrapObject = aPoint._wrapObject;
}

//_____________________________________________________________________________
/**
 * Set the data members of this MuscleWrapPoint to their null values.
 */
void MuscleWrapPoint::setNull()
{
	setType("MuscleWrapPoint");

	_wrapPath.setSize(0);
	_wrapPathLength = 0.0;
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel model containing this MuscleWrapPoint.
 */
void MuscleWrapPoint::setup(Model* aModel, AbstractMuscle* aMuscle)
{
	// base class
	MusclePoint::setup(aModel, aMuscle);
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
MuscleWrapPoint& MuscleWrapPoint::operator=(const MuscleWrapPoint &aPoint)
{
	// BASE CLASS
	MusclePoint::operator=(aPoint);

	copyData(aPoint);

	return(*this);
}

void MuscleWrapPoint::peteTest() const
{
	cout << "   MuscleViaPoint: " << getName() << endl;
	cout << "      point: " << getAttachment() << endl;
	cout << "      body: " << _body->getName() << endl;
}
