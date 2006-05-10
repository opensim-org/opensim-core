// SimmPoint.cpp
// Author: Peter Loan
/* Copyright (c) 2005, Stanford University and Peter Loan.
 * 
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
#include <OpenSim/Tools/rdMath.h>
#include "simmMacros.h"
#include "SimmPoint.h"

//=============================================================================
// STATICS
//=============================================================================


using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SimmPoint::SimmPoint()
{
}

SimmPoint::SimmPoint(double coords[3])
{
	for (int i = 0; i < 3; i++)
		_location[i] = coords[i];
}

SimmPoint::SimmPoint(const SimmPoint& aPoint) :
   Object(aPoint)
{
	for (int i = 0; i < 3; i++)
		_location[i] = aPoint._location[i];
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SimmPoint::~SimmPoint()
{
}

//_____________________________________________________________________________
/**
 * Copy this SimmPoint and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this SimmPoint.
 */
Object* SimmPoint::copy() const
{
	SimmPoint *point = new SimmPoint(*this);
	return(point);
}

SimmPoint& SimmPoint::operator=(const SimmPoint &aPoint)
{
	// BASE CLASS
	Object::operator=(aPoint);

	for (int i = 0; i < 3; i++)
		_location[i] = aPoint._location[i];

	return(*this);
}

SimmPoint& SimmPoint::operator+=(const SimmPoint &aPoint)
{
	for (int i = 0; i < 3; i++)
		_location[i] += aPoint._location[i];

	return(*this);
}

SimmPoint& SimmPoint::operator/=(double factor)
{
	for (int i = 0; i < 3; i++)
		_location[i] /= factor;

	return(*this);
}

void SimmPoint::set(double x, double y, double z)
{
	_location[0] = x;
	_location[1] = y;
	_location[2] = z;
}

/* The point is considered not visible (valid) if any
 * coordinate is NAN.
 */
bool SimmPoint::isVisible() const
{
	if (EQUAL_WITHIN_ERROR(_location[0], rdMath::NAN) ||
		 EQUAL_WITHIN_ERROR(_location[1], rdMath::NAN) ||
		 EQUAL_WITHIN_ERROR(_location[2], rdMath::NAN))
	{
		return false;
	}

	return true;
}

void SimmPoint::peteTest() const
{
	cout << "      SimmPoint: " << _location[0] << ", " << _location[1] << ", " << _location[2] << endl;
}
