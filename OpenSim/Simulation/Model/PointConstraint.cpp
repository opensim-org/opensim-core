// PointConstraint.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
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


//==============================================================================
// INCLUDES
//==============================================================================
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/Mtx.h>
#include "PointConstraint.h"
#include "SimTKcommon.h"

using namespace OpenSim;
using SimTK::Vec3;

//==============================================================================
// CONSTRUCTION
//==============================================================================
//______________________________________________________________________________
/**
 * Destructor.
 */
PointConstraint::~PointConstraint()
{
}
//______________________________________________________________________________
/**
 * Constructor.
 */
PointConstraint::PointConstraint(int aID)
{
	clear();
	_id = aID;
}
//______________________________________________________________________________
/**
 * Constructor.
 */
PointConstraint::
PointConstraint(SimTK::Vec3& aP,SimTK::Vec3& aV,
	SimTK::Vec3& aC0,SimTK::Vec3& aC1,SimTK::Vec3& aC2,int aID)
{
	clear();

	// ID
	_id = aID;

	// POINT
	setPoint(aP);

	// VALUE
	setValue(aV);

	// CONSTRAINTS
	setC0(aC0);
	setC1(aC1);
	setC2(aC2);
}


//==============================================================================
// SET AND GET
//==============================================================================
//------------------------------------------------------------------------------
// NUMBER OF CONSTRAINTS
//------------------------------------------------------------------------------
//______________________________________________________________________________
/**
 * Get the number of non-zero constraint directions.
 *
 * @return Number of constraint directions that have non-zero magnitudes.
 */
int PointConstraint::
getNC()
{
	int nc = 0;
	if(Mtx::Magnitude(3,_c0)!=0) nc++;
	if(Mtx::Magnitude(3,_c1)!=0) nc++;
	if(Mtx::Magnitude(3,_c2)!=0) nc++;

	return(nc);
}

//------------------------------------------------------------------------------
// ID
//------------------------------------------------------------------------------
//______________________________________________________________________________
/**
 * Set the ID of the point constraint.
 *
 * @param aID ID of the point constraint.
 */
void PointConstraint::
setID(int aID)
{
	_id = aID;
}
//______________________________________________________________________________
/**
 * Get the ID of the point.
 *
 * @return ID of the point constraint.
 */
int PointConstraint::
getID()
{
	return(_id);
}

//------------------------------------------------------------------------------
// POINT
//------------------------------------------------------------------------------
//______________________________________________________________________________
/**
 * Set the value of the point.
 */
void PointConstraint::
setPoint(const SimTK::Vec3& aP)
{
	_p = aP;
}
//______________________________________________________________________________
/**
 * Set the value of the point.
 */
void PointConstraint::
setPoint(double aP0,double aP1,double aP2)
{
	_p[0] = aP0;
	_p[1] = aP1;
	_p[2] = aP2;
}
//______________________________________________________________________________
/**
 * Get the value of the point.
 */
void PointConstraint::
getPoint(SimTK::Vec3& aP)
{
	aP = _p;
}
//______________________________________________________________________________
/**
 * Get the value of the point.
 */
Vec3& PointConstraint::
getPoint()
{
	return(_p);
}

//------------------------------------------------------------------------------
// VALUE
//------------------------------------------------------------------------------
//______________________________________________________________________________
/**
 * Set the value of the constraint.
 */
void PointConstraint::
setValue(const SimTK::Vec3& aV)
{
	_v = aV;
}
//______________________________________________________________________________
/**
 * Set the value of the constraint.
 */
void PointConstraint::
setValue(double aV0,double aV1,double aV2)
{
	_v[0] = aV0;
	_v[1] = aV1;
	_v[2] = aV2;
}
//______________________________________________________________________________
/**
 * Get the value of the constraint.
 */
void PointConstraint::
getValue(SimTK::Vec3& aV)
{
	aV = _v;
}
//______________________________________________________________________________
/**
 * Get the value of the constraint.
 */
Vec3& PointConstraint::
getValue()
{
	return(_v);
}

//------------------------------------------------------------------------------
// CONSTRAINT DIRECTION 0
//------------------------------------------------------------------------------
//______________________________________________________________________________
/**
 * Set constraint direction 0.
 */
void PointConstraint::
setC0(const SimTK::Vec3& aC)
{
	_c0 = aC;
}
//______________________________________________________________________________
/**
 * Set constraint direction 0.
 */
void PointConstraint::
setC0(double aC0,double aC1,double aC2)
{
	_c0[0] = aC0;
	_c0[1] = aC1;
	_c0[2] = aC2;
}
//______________________________________________________________________________
/**
 * Get the constraint direction.
 */
void PointConstraint::
getC0(SimTK::Vec3& aC)
{
	aC = _c0;
}
//______________________________________________________________________________
/**
 * Get the constraint direction.
 */
Vec3& PointConstraint::
getC0()
{
	return(_c0);
}

//------------------------------------------------------------------------------
// CONSTRAINT DIRECTION 1
//------------------------------------------------------------------------------
//______________________________________________________________________________
/**
 * Set the constraint direction.
 */
void PointConstraint::
setC1(const SimTK::Vec3& aC)
{
	_c1 = aC;
}
//______________________________________________________________________________
/**
 * Set the constraint direction.
 */
void PointConstraint::
setC1(double aC0,double aC1,double aC2)
{
	_c1[0] = aC0;
	_c1[1] = aC1;
	_c1[2] = aC2;
}
//______________________________________________________________________________
/**
 * Get the constraint direction.
 */
void PointConstraint::
getC1(SimTK::Vec3& aC)
{
	aC = _c1;
}
//______________________________________________________________________________
/**
 * Get the constraint direction.
 */
Vec3& PointConstraint::
getC1()
{
	return(_c1);
}

//------------------------------------------------------------------------------
// CONSTRAINT DIRECTION 2
//------------------------------------------------------------------------------
//______________________________________________________________________________
/**
 * Set the constraint direction.
 */
void PointConstraint::
setC2(const SimTK::Vec3& aC)
{
	_c2 = aC;
}
//______________________________________________________________________________
/**
 * Set the constraint direction.
 */
void PointConstraint::
setC2(double aC0,double aC1,double aC2)
{
	_c2[0] = aC0;
	_c2[1] = aC1;
	_c2[2] = aC2;
}
//______________________________________________________________________________
/**
 * Get the constraint direction.
 */
void PointConstraint::
getC2(SimTK::Vec3& aC)
{
	aC = _c2;
}
//______________________________________________________________________________
/**
 * Get the constraint direction.
 */
Vec3& PointConstraint::
getC2()
{
	return(_c2);
}


//==============================================================================
// EVALUATE THE CONSTRAINTS
//==============================================================================
//______________________________________________________________________________
/**
 * Evaluate the point constraint in direction C0.
 *
 * @param aV Vector to be dotted with C0.
 * @return Dot product of (aV - Value) with C0.
 */
double PointConstraint::
evaluateC0(SimTK::Vec3& aV)
{
	SimTK::Vec3 v = aV - _v;
	double result = Mtx::DotProduct(3,_c0,v);
	return(result);
}
//______________________________________________________________________________
/**
 * Evaluate the point constraint in direction C1.
 *
 * @param aV Vector to be dotted with C1.
 * @return Dot product of (aV - Value) with C0.
 */
double PointConstraint::
evaluateC1(SimTK::Vec3& aV)
{
	SimTK::Vec3 v = aV - _v;
	double result = Mtx::DotProduct(3,_c1,v);
	return(result);
}
//______________________________________________________________________________
/**
 * Evaluate the point constraint in direction C2.
 *
 * @param aV Vector to be dotted with C2.
 * @return Dot product of (aV - Value) with C0.
 */
double PointConstraint::
evaluateC2(SimTK::Vec3& aV)
{
	SimTK::Vec3 v = aV - _v;
	double result = Mtx::DotProduct(3,_c2,v);
	return(result);
}


//==============================================================================
// UTILITY
//==============================================================================
//______________________________________________________________________________
/**
 * Construct a set of orthonormal constraint directions based on a provided
 * vector and C0 such that C0 = C0/|C0|, C1 = (aV x aC0)/|aV|, and
 * C2 = C0 x C1.
 *
 * If any vector cannot be constructed validly, it is set to the zero vector.
 *
 * @param aV Provided vector.
 * @param aC0 Direction for C0.
 */
void PointConstraint::
constructOrthoNormalConstraints(SimTK::Vec3& aV,SimTK::Vec3& aC0)
{
	zeroConstraints();
	setC0(aC0);  Mtx::Normalize(3,_c0,_c0);
	constructOrthoNormalC1(aV);
	constructOrthoNormalC2();
}
//______________________________________________________________________________
/**
 * Construct constraint direction C1 so that it is normal and orthogonal to C0
 * and a specified vector acording to C1 = (aV x aC0) / (|aV|*|aC0|).
 *
 * If either aV or C0 are not valid, C1 is set to a zero vector.
 *
 * @param aV Specified vector.
 */
void PointConstraint::
constructOrthoNormalC1(SimTK::Vec3& aV)
{
	setC1(0.0,0.0,0.0);
	Mtx::CrossProduct(aV,_c0,_c1);
	Mtx::Normalize(3,_c1,_c1);
}
//______________________________________________________________________________
/**
 * Construct constraint direction C2 so that it is normal and orthogonal to C0
 * and C1.
 *
 * Note that if C0 or C1 are parallel or if either has zero magnitude, C2
 * will be set to the zero vector.
 */
void PointConstraint::
constructOrthoNormalC2()
{
	setC2(0.0,0.0,0.0);
	Mtx::CrossProduct(_c0,_c1,_c2);
	Mtx::Normalize(3,_c2,_c2);
}
//______________________________________________________________________________
/**
 * Normalize all three constraint directions.
 */
void PointConstraint::
normalizeConstraints()
{
	Mtx::Normalize(3,_c0,_c0);
	Mtx::Normalize(3,_c1,_c1);
	Mtx::Normalize(3,_c2,_c2);
}
//______________________________________________________________________________
/**
 * Zero all three constraint directions.
 */
void PointConstraint::
zeroConstraints()
{
	setC0(0.0,0.0,0.0);
	setC1(0.0,0.0,0.0);
	setC2(0.0,0.0,0.0);
}
//______________________________________________________________________________
/**
 * Clear this point constraint.
 *
 * The ID is set to 0, the point is set to the zero vector, and all constraint
 * directions are set to the zero vector.
 */
void PointConstraint::
clear()
{
	setID(0);
	setPoint(0.0,0.0,0.0);
	setValue(0.0,0.0,0.0);
	setC0(0.0,0.0,0.0);
	setC1(0.0,0.0,0.0);
	setC2(0.0,0.0,0.0);
}
