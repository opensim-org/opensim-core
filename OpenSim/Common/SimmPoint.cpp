// SimmPoint.cpp
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
#include "rdMath.h"
#include "SimmMacros.h"
#include "SimmPoint.h"

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
SimmPoint::SimmPoint()
{
	setNull();
}

//_____________________________________________________________________________
/**
 * Constructor from a set of XYZ coordinates.
 */
SimmPoint::SimmPoint(const SimTK::Vec3& coords)
{
	setNull();

	_location = coords;
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aPoint SimmPoint to be copied.
 */
SimmPoint::SimmPoint(const SimmPoint& aPoint) :
   Object(aPoint)
{
	setNull();

	_location = aPoint._location;
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

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL values for all the variable members of this class.
 */
void SimmPoint::setNull()
{
	setType("SimmPoint");
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
SimmPoint& SimmPoint::operator=(const SimmPoint &aPoint)
{
	// BASE CLASS
	Object::operator=(aPoint);

	_location = aPoint._location;

	return(*this);
}

//_____________________________________________________________________________
/**
 * Plus-equals operator.
 *
 * @return Reference to this object.
 */
SimmPoint& SimmPoint::operator+=(const SimmPoint &aPoint)
{
	_location += aPoint._location;

	return(*this);
}

//_____________________________________________________________________________
/**
 * Divide-equals operator.
 *
 * @return Reference to this object.
 */
SimmPoint& SimmPoint::operator/=(double factor)
{
	_location /= factor;

	return(*this);
}

//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the coordinates of the SimmPoint
 *
 * @param x X coordinate
 * @param y Y coordinate
 * @param z Z coordinate
 */
void SimmPoint::set(double x, double y, double z)
{
	_location[0] = x;
	_location[1] = y;
	_location[2] = z;
}

//_____________________________________________________________________________
/**
 * Scale the XYZ coordinates by a scalar.
 *
 * @param The scale factor.
 */
void SimmPoint::scale(double aScaleFactor)
{
	_location *= aScaleFactor;
}

//_____________________________________________________________________________
/**
 * Is the SimmPoint visible? It is considered invisible if
 * any coordinate is NAN.
 *
 * @return Whether or not the SimmPoint is visible.
 */
bool SimmPoint::isVisible() const
{
	if (rdMath::isNAN(_location[0]) ||
		 rdMath::isNAN(_location[1]) ||
		 rdMath::isNAN(_location[2]))
	{
		return false;
	}

	return true;
}
