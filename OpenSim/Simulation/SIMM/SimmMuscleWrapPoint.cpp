// SimmMuscleWrapPoint.cpp
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
#include "SimmMuscleWrapPoint.h"
#include "AbstractModel.h"
#include "AbstractSimmMuscle.h"
#include "AbstractBody.h"
#include "AbstractCoordinate.h"
#include "CoordinateSet.h"
#include "AbstractDynamicsEngine.h"

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
SimmMuscleWrapPoint::SimmMuscleWrapPoint()
{
	setNull();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SimmMuscleWrapPoint::~SimmMuscleWrapPoint()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aPoint SimmMuscleWrapPoint to be copied.
 */
SimmMuscleWrapPoint::SimmMuscleWrapPoint(const SimmMuscleWrapPoint &aPoint) :
   SimmMusclePoint(aPoint)
{
	setNull();
	copyData(aPoint);
}

//_____________________________________________________________________________
/**
 * Copy this muscle via point and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this SimmMuscleWrapPoint.
 */
Object* SimmMuscleWrapPoint::copy() const
{
	SimmMuscleWrapPoint *pt = new SimmMuscleWrapPoint(*this);
	return(pt);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one SimmMuscleWrapPoint to another.
 *
 * @param aPoint SimmMuscleWrapPoint to be copied.
 */
void SimmMuscleWrapPoint::copyData(const SimmMuscleWrapPoint &aPoint)
{
}

//_____________________________________________________________________________
/**
 * Set the data members of this SimmMuscleWrapPoint to their null values.
 */
void SimmMuscleWrapPoint::setNull()
{
	setType("SimmMuscleWrapPoint");
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel model containing this SimmMuscleWrapPoint.
 */
void SimmMuscleWrapPoint::setup(AbstractModel* aModel, AbstractSimmMuscle* aMuscle)
{
	// base class
	SimmMusclePoint::setup(aModel, aMuscle);
}

double SimmMuscleWrapPoint::getWrapLength() const
{
	return 0.0;
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
SimmMuscleWrapPoint& SimmMuscleWrapPoint::operator=(const SimmMuscleWrapPoint &aPoint)
{
	// BASE CLASS
	SimmMusclePoint::operator=(aPoint);

	copyData(aPoint);

	return(*this);
}

void SimmMuscleWrapPoint::peteTest() const
{
	cout << "   MuscleViaPoint: " << getName() << endl;
	cout << "      point: " << getAttachment() << endl;
	cout << "      body: " << _body->getName() << endl;
}
