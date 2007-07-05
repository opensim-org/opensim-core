// Function.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c) 2005, Stanford University. All rights reserved. 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions
* are met: 
*  - Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer. 
*  - Redistributions in binary form must reproduce the above copyright 
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the distribution. 
*  - Neither the name of the Stanford University nor the names of its 
*    contributors may be used to endorse or promote products derived 
*    from this software without specific prior written permission. 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
* POSSIBILITY OF SUCH DAMAGE. 
*/

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


// INCLUDES
#include "Function.h"
#include "PropertyDbl.h"
#include "rdMath.h"


using namespace OpenSim;
using namespace std;


//=============================================================================
// STATICS
//=============================================================================


//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
Function::~Function()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
Function::Function() :
	_minX(_propMinX.getValueDbl()),
	_maxX(_propMaxX.getValueDbl()),
	_minY(_propMinY.getValueDbl()),
	_maxY(_propMaxY.getValueDbl()),
	_minZ(_propMinZ.getValueDbl()),
	_maxZ(_propMaxZ.getValueDbl())
{
	setNull();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aFunction Function to copy.
 */
Function::Function(const Function &aFunction) :
	Object(aFunction),
	_minX(_propMinX.getValueDbl()),
	_maxX(_propMaxX.getValueDbl()),
	_minY(_propMinY.getValueDbl()),
	_maxY(_propMaxY.getValueDbl()),
	_minZ(_propMinZ.getValueDbl()),
	_maxZ(_propMaxZ.getValueDbl())
{
	setupProperties();

	// ASSIGN
	setEqual(aFunction);
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set all member variables to NULL values.
 */
void Function::
setNull()
{
	setType("Function");
	setupProperties();
}
//_____________________________________________________________________________
/**
 * Set up the serialized member variables.  This involves both generating
 * the properties and connecting them to the local pointers used to access
 * the serialized member variables.
 */
void Function::
setupProperties()
{
	// X
	_propMinX.setName("min_x");
	_propMinX.setValue(0.0);
	_propertySet.append( &_propMinX );

	_propMaxX.setName("max_x");
	_propMaxX.setValue(0.0);
	_propertySet.append( &_propMaxX );

	// Y
	_propMinY.setName("min_y");
	_propMinY.setValue(0.0);
	_propertySet.append( &_propMinY );

	_propMaxY.setName("max_y");
	_propMaxY.setValue(0.0);
	_propertySet.append( &_propMaxY );


	// Z
	_propMinZ.setName("min_z");
	_propMinZ.setValue(0.0);
	_propertySet.append( &_propMinZ );

	_propMaxZ.setName("max_z");
	_propMaxZ.setValue(0.0);
	_propertySet.append( &_propMaxZ );

}

//_____________________________________________________________________________
/**
 * Set all member variables equal to the members of another object.
 * Note that this method is private.  It is only meant for copying the data
 * members defined in this class.  It does not, for example, make any changes
 * to data members of base classes.
 */
void Function::
setEqual(const Function &aFunction)
{
	setMinX(aFunction.getMinX());
	setMinY(aFunction.getMinY());
	setMinZ(aFunction.getMinZ());
	setMaxX(aFunction.getMaxX());
	setMaxY(aFunction.getMaxY());
	setMaxZ(aFunction.getMaxZ());
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
Function& Function::
operator=(const Function &aFunction)
{
	// BASE CLASS
	Object::operator=(aFunction);

	// DATA
	setEqual(aFunction);

	return(*this);
}


//=============================================================================
// SET AND GET
//=============================================================================
//-----------------------------------------------------------------------------
// MIN AND MAX X
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the minimum x independent variable.
 *
 * @param aMinX Minimum x.
 */
void Function::
setMinX(double aMinX)
{
	_minX = aMinX;
}
//_____________________________________________________________________________
/**
 * Get the minimum x independent variable.
 *
 * @return Minimum x.
 */
double Function::
getMinX() const
{
	return(_minX);
}

//_____________________________________________________________________________
/**
 * Set the maximum x independent variable.
 *
 * @param aMaxX Maximum x.
 */
void Function::
setMaxX(double aMaxX)
{
	_maxX = aMaxX;
}
//_____________________________________________________________________________
/**
 * Get the maximum x independent variable.
 *
 * @return Maximum x.
 */
double Function::
getMaxX() const
{
	return(_maxX);
}

//-----------------------------------------------------------------------------
// MIN AND MAX Y
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the minimum y independent variable.
 *
 * @param aMinY Minimum y.
 */
void Function::
setMinY(double aMinY)
{
	_minY = aMinY;
}
//_____________________________________________________________________________
/**
 * Get the minimum y independent variable.
 *
 * @return Minimum y.
 */
double Function::
getMinY() const
{
	return(_minY);
}

//_____________________________________________________________________________
/**
 * Set the maximum y independent variable.
 *
 * @param aMaxY Maximum y.
 */
void Function::
setMaxY(double aMaxY)
{
	_maxY = aMaxY;
}
//_____________________________________________________________________________
/**
 * Get the maximum y independent variable.
 *
 * @return Maximum y.
 */
double Function::
getMaxY() const
{
	return(_maxY);
}

//-----------------------------------------------------------------------------
// MIN AND MAX Z
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the minimum z independent variable.
 *
 * @param aMinZ Minimum z.
 */
void Function::
setMinZ(double aMinZ)
{
	_minZ = aMinZ;
}
//_____________________________________________________________________________
/**
 * Get the minimum z independent variable.
 *
 * @return Minimum z.
 */
double Function::
getMinZ() const
{
	return(_minZ);
}

//_____________________________________________________________________________
/**
 * Set the maximum z independent variable.
 *
 * @param aMaxZ Maximum z.
 */
void Function::
setMaxZ(double aMaxZ)
{
	_maxZ = aMaxZ;
}
//_____________________________________________________________________________
/**
 * Get the maximum z independent variable.
 *
 * @return Maximum z.
 */
double Function::
getMaxZ() const
{
	return(_maxZ);
}


//=============================================================================
// UTILITIES
//=============================================================================
//_____________________________________________________________________________
/**
 * Determine if a function is approixmately linear over a region of interest.
 * The implementation in the base class is crude and does not guarrantee that
 * the function is linear thoughout the specified range. It uses a number of
 * numerical evaluations to determine whether or not the slopes with respect
 * to the independent variables are constant throughout the specified range
 * of the function.
 *
 * Derived classes should implement a more conclusive method if possible.
 *
 * @param aTol Tolerance for being a line.  If the slope of the function
 * is the same to within this tolerance throughout the specified range,
 * the function is considered a line.
 * @param aMinX Minimum of X.
 * @param aMaxX Maximum of X.
 * @param rMX Slope of the function wrt to X.
 * @param aMinY Minimum of Y.
 * @param aMaxY Maximum of Y.
 * @param rMY Slope of the function wrt to Y.
 * @param aMinZ Minimum of Z.
 * @param aMaxZ Maximum of Z.
 * @param rMZ Slope of the function wrt to Z.
 * If the function is not a linear wrt an indpendent variable, NAN is
 * returned for the corresponding slope.
 */
void Function::isLinear(double aTol,
								double aMinX,double aMaxX,double &rMX,
								double aMinY,double aMaxY,double &rMY,
								double aMinZ,double aMaxZ,double &rMZ)
{
	double xmid = 0.5*(aMinX - aMaxX);
	double ymid = 0.5*(aMinY - aMaxY);
	double zmid = 0.5*(aMinZ - aMaxZ);
	double m1,m2,m3;

	// X
	m1 = evaluate(1,aMinX,ymid,zmid);
	m2 = evaluate(1,xmid,ymid,zmid);
	m3 = evaluate(1,aMaxX,ymid,zmid);
	if(rdMath::IsEqual(m1,m2,aTol) && rdMath::IsEqual(m2,m3,aTol) && rdMath::IsEqual(m3,m1,aTol)) {
		rMX = m2;
	} else {
		rMX = rdMath::NAN;
	}
	// Y
	m1 = evaluate(1,xmid,aMinY,zmid);
	m2 = evaluate(1,xmid,ymid,zmid);
	m3 = evaluate(1,xmid,aMaxY,zmid);
	if(rdMath::IsEqual(m1,m2,aTol) && rdMath::IsEqual(m2,m3,aTol) && rdMath::IsEqual(m3,m1,aTol)) {
		rMY = m2;
	} else {
		rMY = rdMath::NAN;
	}
	// Z
	m1 = evaluate(1,xmid,ymid,aMinZ);
	m2 = evaluate(1,xmid,ymid,zmid);
	m3 = evaluate(1,xmid,ymid,aMaxZ);
	if(rdMath::IsEqual(m1,m2,aTol) && rdMath::IsEqual(m2,m3,aTol) && rdMath::IsEqual(m3,m1,aTol)) {
		rMZ = m2;
	} else {
		rMZ = rdMath::NAN;
	}
}

//=============================================================================
// EVALUATE
//=============================================================================
/**
 * Evaluates total first derivative using the chain rule.
 */
double Function::
evaluateTotalFirstDerivative(double aX,double aDxdt)
{
	return evaluate(1,aX) * aDxdt;
}

/**
 * Evaluates total second derivative using the chain rule.
 */
double Function::
evaluateTotalSecondDerivative(double aX,double aDxdt,double aD2xdt2)
{
	return evaluate(1,aX) * aD2xdt2 + evaluate(2,aX) * aDxdt * aDxdt;
}
