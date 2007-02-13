// Body.cpp
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

#include <math.h>
#include "Body.h"
#include <OpenSim/Tools/PropertyDbl.h>
#include <OpenSim/Tools/PropertyDblArray.h>


using namespace std;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */

Body::Body(double aM,double *aI):
PolyObject(),
_M(_propM.getValueDbl()),
_I(_propI.getValueDblArray()),
_centerOfMass(_propCenterOfMass.getValueDblArray())
{
	setNull();

	// TYPE
	setType("Body");

	setMass(aM);
	setInertia(aI);

}
//_____________________________________________________________________________
/**
 * Construct an object from file.
 *
 * The object is constructed from the root element of the XML document.
 * The type of object is the tag name of the XML root element.
 *
 * @param aFileName File name of the document.
 */
Body::Body(const string &aFileName):
PolyObject(aFileName),
_M(_propM.getValueDbl()),
_I(_propI.getValueDblArray()),
_centerOfMass(_propCenterOfMass.getValueDblArray())
{
	setNull();
	updateFromXMLNode();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aBody Body to copy.
 */
Body::Body(const Body &aBody) :
PolyObject(aBody),
_M(_propM.getValueDbl()),
_I(_propI.getValueDblArray()),
_centerOfMass(_propCenterOfMass.getValueDblArray())
{
	setNull();

	// ASSIGN
	*this = aBody;
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
Body::~Body()
{

}
//_____________________________________________________________________________
/**
 * Another incarnation of copy that's virtual.
 *
 * @param aElement XMLnode to construct body from.
 */

Object *Body::
copy() const
{
	return(new Body(*this));
}

//_____________________________________________________________________________
/**
 * Set all member variables to their null or default values.
 */
void Body::
setNull()
{

	setupProperties();

}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void Body::
setupProperties()
{
	Array<double> zeroVector(0.0, 6);
	Array<double> zero3(0.0, 3);	
	
	_propertySet.append(&_propM);
	_propM.setName("mass");
	_propM.setValue(1.0);

	_propertySet.append(&_propI);
	_propI.setName("inertia");
	_propI.setValue(zeroVector);

	_propertySet.append(&_propCenterOfMass);
	_propCenterOfMass.setName("center_of_mass");
	_propCenterOfMass.setValue(zero3);

}


//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// ASSIGNMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Assign this object to the values of another.
 *
 * @return Reference to this object.
 */
Body& Body::
operator=(const Body &aObject)
{
	// BASE CLASS
	VisibleObject::operator=(aObject);

	// Class Members
	_M = (aObject._M);
	_I = (aObject._I);

	_centerOfMass = aObject._centerOfMass;

	return(*this);
}

//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// MASS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the mass.
 * The mass must be greater than or equal to zero.
 */
void Body::
setMass(double aM)
{
	_M = fabs(aM);
}
//_____________________________________________________________________________
/**
 * Get the mass.
 */
double Body::
getMass()
{
	return(_M);
}

//-----------------------------------------------------------------------------
// INERTIA SCALARS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the inertia scalars.
 *
 * The argument aI is assumed to point to a array of values which has as its
 * first six values I1, I2, I3, I12, I23, I31.
 *
 * If aI is NULL, then the inertia scalars are set to represent the inertia
 * matrix for a sphere of uniform density of the same mass as the body
 */
void Body::
setInertia(double *aI)
{
	int i;
	if(aI==NULL) {
		double I = (_M)*2.0/5.0;
		for(i=0;i<3;i++) _I[i] = I;
		for(i=3;i<6;i++) _I[i] = 0.0;
	} else {
		for(i=0;i<6;i++) _I[i] = aI[i];
	}
}
//_____________________________________________________________________________
/**
 * Set the inertia scalars.
 *
 * aI1, aI2, aI3 correspond to the moments of inertia.  It is assumed that
 * the products of inertia are 0.0.
 */
void Body::
setInertia(double aI1,double aI2,double aI3)
{
	_I[0] = aI1;
	_I[1] = aI2;
	_I[2] = aI3;
	_I[3] = 0.0;
	_I[4] = 0.0;
	_I[5] = 0.0;
}
//_____________________________________________________________________________
/**
 * Set the inertia scalars.
 */
void Body::
setInertia(double aI1,double aI2,double aI3,
	double aI12,double aI23,double aI31)
{
	_I[0] = aI1;
	_I[1] = aI2;
	_I[2] = aI3;
	_I[3] = aI12;
	_I[4] = aI23;
	_I[5] = aI31;
}
//_____________________________________________________________________________
/**
 * Get the inertia scalars.
 *
 * The argument aI is filled with the values aI1, aI2, aI3, aI12, aI23, aI31.
 */
void Body::
getInertia(double aI[6])
{
	int i;
	for(i=0;i<6;i++) aI[i]=_I[i];
}
//_____________________________________________________________________________
/**
 * Get the inertia scalars.
 *
 * The full scalar inertia matrix is contructed.
 */
void Body::
getInertia(double aI[3][3])
{
	// MOMENTS
	aI[0][0] = _I[0];
	aI[1][1] = _I[1];
	aI[2][2] = _I[2];

	// PRODUCTS
	aI[0][1] = aI[1][0] = _I[3];
	aI[1][2] = aI[2][1] = _I[4];
	aI[2][0] = aI[0][2] = _I[5];
}
//_____________________________________________________________________________
/**
 * Set Center of mass relative to geometry's CoordinateSystem.
 *
 */
void Body::
setCenterOfMass(const double aCenterOfMass[3])
{
	for(int i=0; i < 3; i++)
		_centerOfMass[i]=(aCenterOfMass[i]);
}
//_____________________________________________________________________________
/**
 * Get Center of mass relative to geometry's CoordinateSystem.
 *
 */
void Body::
getCenterOfMass(double aCenterOfMass[3]) const
{
	for(int i=0; i < 3; i++)
		aCenterOfMass[i] = _centerOfMass[i];
}
//--------------------------------------------------------------------------
// SCALING
//--------------------------------------------------------------------------
void Body::
scaleBy(const double aScaleFactors[3])
{
	// Scale center of mass
	for(int i=0; i < 3; i++)
		_centerOfMass[i] *= aScaleFactors[i];
	// Compute mass_scale based on Volume, assume fixed density
	double massScale = aScaleFactors[0]*aScaleFactors[1]*aScaleFactors[2];
	// Scale mass
	_M *= massScale;
	// Scale inertia vector
	double inertia[3][3];
	getInertia(inertia);
	for (int j=0;j<3;j++) {
		for (int k=0;k<3;k++) {
			inertia[j][k]*= massScale*aScaleFactors[j]*aScaleFactors[k];
		}
	}
	setInertia(inertia[0][0], inertia[1][1], inertia[2][2],
				 inertia[0][1],  inertia[1][2],  inertia[2][0]);
	// Scale geometry for display purposes
	setScaleFactors(aScaleFactors);


}
