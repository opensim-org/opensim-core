// PointConstraint.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//==============================================================================
// INCLUDES
//==============================================================================
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "Math.h"
#include "Mtx.h"
#include "PointConstraint.h"



//==============================================================================
// CONSTRUCTION
//==============================================================================
//______________________________________________________________________________


using namespace OpenSim;
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
PointConstraint(double aP[3],double aV[3],
	double aC0[3],double aC1[3],double aC2[3],int aID)
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
setPoint(double aP[3])
{
	_p[0] = aP[0];
	_p[1] = aP[1];
	_p[2] = aP[2];
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
getPoint(double aP[3])
{
	aP[0] = _p[0];
	aP[1] = _p[1];
	aP[2] = _p[2];
}
//______________________________________________________________________________
/**
 * Get the value of the point.
 */
double* PointConstraint::
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
setValue(double aV[3])
{
	_v[0] = aV[0];
	_v[1] = aV[1];
	_v[2] = aV[2];
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
getValue(double aV[3])
{
	aV[0] = _v[0];
	aV[1] = _v[1];
	aV[2] = _v[2];
}
//______________________________________________________________________________
/**
 * Get the value of the constraint.
 */
double* PointConstraint::
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
setC0(double aC[3])
{
	_c0[0] = aC[0];
	_c0[1] = aC[1];
	_c0[2] = aC[2];
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
getC0(double aC[3])
{
	aC[0] = _c0[0];
	aC[1] = _c0[1];
	aC[2] = _c0[2];
}
//______________________________________________________________________________
/**
 * Get the constraint direction.
 */
double* PointConstraint::
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
setC1(double aC[3])
{
	_c1[0] = aC[0];
	_c1[1] = aC[1];
	_c1[2] = aC[2];
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
getC1(double aC[3])
{
	aC[0] = _c1[0];
	aC[1] = _c1[1];
	aC[2] = _c1[2];
}
//______________________________________________________________________________
/**
 * Get the constraint direction.
 */
double* PointConstraint::
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
setC2(double aC[3])
{
	_c2[0] = aC[0];
	_c2[1] = aC[1];
	_c2[2] = aC[2];
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
getC2(double aC[3])
{
	aC[0] = _c2[0];
	aC[1] = _c2[1];
	aC[2] = _c2[2];
}
//______________________________________________________________________________
/**
 * Get the constraint direction.
 */
double* PointConstraint::
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
evaluateC0(double aV[3])
{
	double v[3];
	Mtx::Subtract(1,3,aV,_v,v);
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
evaluateC1(double aV[3])
{
	double v[3];
	Mtx::Subtract(1,3,aV,_v,v);
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
evaluateC2(double aV[3])
{
	double v[3];
	Mtx::Subtract(1,3,aV,_v,v);
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
constructOrthoNormalConstraints(double aV[3],double aC0[3])
{
	zeroConstraints();
	if((aV==NULL)||(aC0==NULL)) return;
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
constructOrthoNormalC1(double aV[3])
{
	setC1(0.0,0.0,0.0);
	if(aV==NULL) return;
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
