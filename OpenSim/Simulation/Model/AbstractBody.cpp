// AbstractBody.cpp
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
#include "AbstractBody.h"
#include "AbstractDynamicsEngine.h"
#include <OpenSim/Common/SimmMacros.h>

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
AbstractBody::AbstractBody() :
	Object(),
	_wrapObjectSetProp(PropertyObj("", WrapObjectSet())),
	_wrapObjectSet((WrapObjectSet&)_wrapObjectSetProp.getValueObj())
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
AbstractBody::~AbstractBody()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aBody AbstractBody to be copied.
 */
AbstractBody::AbstractBody(const AbstractBody &aBody) :
	Object(aBody),
	_wrapObjectSetProp(PropertyObj("", WrapObjectSet())),
	_wrapObjectSet((WrapObjectSet&)_wrapObjectSetProp.getValueObj())
{
	setNull();
	setupProperties();
	copyData(aBody);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one AbstractBody to another.
 *
 * @param aBody AbstractBody to be copied.
 */
void AbstractBody::copyData(const AbstractBody &aBody)
{
	_dynamicsEngine = aBody._dynamicsEngine;
	_wrapObjectSet = aBody._wrapObjectSet;
}

/**
 * Set the data members of this AbstractBody to their null values.
 */
void AbstractBody::setNull()
{
	setType("AbstractBody");

	_dynamicsEngine = NULL;
}

//_____________________________________________________________________________
/**
 * Perform set up functions after model has been deserialized or copied.
 *
 * @param aEngine dynamics engine containing this body.
 */
void AbstractBody::setup(AbstractDynamicsEngine* aEngine)
{
	_dynamicsEngine = aEngine;

	int i;
	for (i = 0; i < _wrapObjectSet.getSize(); i++)
		_wrapObjectSet.get(i)->setup(aEngine, this);
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 *
 */
void AbstractBody::setupProperties()
{
	_wrapObjectSetProp.setName("WrapObjectSet");
	_propertySet.append(&_wrapObjectSetProp);
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
AbstractBody& AbstractBody::operator=(const AbstractBody &aBody)
{
	// BASE CLASS
	Object::operator=(aBody);

	copyData(aBody);

	return(*this);
}

//_____________________________________________________________________________
/**
 * Get the named wrap object, if it exists.
 *
 * @param aName Name of the wrap object.
 * @return Pointer to the wrap object.
 */
AbstractWrapObject* AbstractBody::getWrapObject(const string& aName) const
{
	int i;

	for (i = 0; i < _wrapObjectSet.getSize(); i++) {
		if (aName == _wrapObjectSet.get(i)->getName())
			return _wrapObjectSet.get(i);
	}

	return NULL;
}

//_____________________________________________________________________________
/**
 * Computes scaled inertia tensor (assuming mass stays fixed -- i.e. the density changes!)
 */
void AbstractBody::scaleInertiaTensor(double aMass, const Array<double> &aScaleFactors, double rInertia[3][3])
{
	/* If the mass is zero, then make the inertia tensor zero as well.
	 * If the X, Y, Z scale factors are equal, then you can scale the
	 * rInertia tensor exactly by the square of the scale factor, since
	 * each element in the tensor is proportional to the square of one
	 * or more dimensional measurements. For determining if the scale
	 * factors are equal, ignore reflections-- look only at the
	 * absolute value of the factors.
	 */
	if (aMass <= ROUNDOFF_ERROR) {
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				rInertia[i][j] = 0.0;

	} else if (EQUAL_WITHIN_ERROR(DABS(aScaleFactors[0]), DABS(aScaleFactors[1])) &&
		      EQUAL_WITHIN_ERROR(DABS(aScaleFactors[1]), DABS(aScaleFactors[2]))) {
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				rInertia[i][j] *= (aScaleFactors[0] * aScaleFactors[0]);

	} else {
		/* If the scale factors are not equal, then assume that the segment
		 * is a cylinder and the rInertia is calculated about one end of it.
		 */
		int axis;

		/* 1. Find the smallest diagonal component. This dimension is the axis
		 *    of the cylinder.
		 */
		if (rInertia[0][0] <= rInertia[1][1]){
			if (rInertia[0][0] <= rInertia[2][2])
				axis = 0;
			else
				axis = 2;

		} else if (rInertia[1][1] <= rInertia[2][2]) {
			axis = 1;

		} else {
			axis = 2;
		}

		/* 2. The smallest rInertial component is equal to 0.5 * mass * radius * radius,
		 *    so you can rearrange and solve for the radius.
		 */
		int oa;
		double radius, rad_sqr, length;
		double term = 2.0 * rInertia[axis][axis] / aMass;
		if (term < 0.0)
			radius = 0.0;
		else
			radius = sqrt(term);

		/* 3. Choose either of the other diagonal components and use it to solve for the
		*    length of the cylinder. This component is equal to:
		*    0.333 * aMass * length * length  +  0.25 * aMass * radius * radius
		*/
		if (axis == 0)
			oa = 1;
		else
			oa = 0;
		term = 3.0 * (rInertia[oa][oa] - 0.25 * aMass * radius * radius) / aMass;
		if (term < 0.0)
			length = 0.0;
		else
			length = sqrt(term);

		/* 4. Scale the radius, and length, and recalculate the diagonal rInertial terms. */
		length *= DABS(aScaleFactors[axis]);

		if (axis == 0) {
			rad_sqr = radius * DABS(aScaleFactors[1]) * radius * DABS(aScaleFactors[2]);
			rInertia[0][0] = 0.5 * aMass * rad_sqr;
			rInertia[1][1] = aMass * ((length * length / 3.0) + 0.25 * rad_sqr);
			rInertia[2][2] = aMass * ((length * length / 3.0) + 0.25 * rad_sqr);

		} else if (axis == 1) {
			rad_sqr = radius * DABS(aScaleFactors[0]) * radius * DABS(aScaleFactors[2]);
			rInertia[0][0] = aMass * ((length * length / 3.0) + 0.25 * rad_sqr);
			rInertia[1][1] = 0.5 * aMass * rad_sqr;
			rInertia[2][2] = aMass * ((length * length / 3.0) + 0.25 * rad_sqr);

		} else {
			rad_sqr = radius * DABS(aScaleFactors[0]) * radius * DABS(aScaleFactors[1]);
			rInertia[0][0] = aMass * ((length * length / 3.0) + 0.25 * rad_sqr);
			rInertia[1][1] = aMass * ((length * length / 3.0) + 0.25 * rad_sqr);
			rInertia[2][2] = 0.5 * aMass * rad_sqr;
		}

		/* 5. Scale the cross terms, in case some are non-zero. */
		rInertia[0][1] *= DABS((aScaleFactors[0] * aScaleFactors[1]));
		rInertia[0][2] *= DABS((aScaleFactors[0] * aScaleFactors[2]));
		rInertia[1][0] *= DABS((aScaleFactors[1] * aScaleFactors[0]));
		rInertia[1][2] *= DABS((aScaleFactors[1] * aScaleFactors[2]));
		rInertia[2][0] *= DABS((aScaleFactors[2] * aScaleFactors[0]));
		rInertia[2][1] *= DABS((aScaleFactors[2] * aScaleFactors[1]));
	}
}
