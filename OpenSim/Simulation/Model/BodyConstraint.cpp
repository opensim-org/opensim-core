// BodyConstraint.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



//==============================================================================
// INCLUDES
//==============================================================================
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/Mtx.h>
#include "PointConstraint.h"
#include "BodyConstraint.h"


using namespace OpenSim;


//==============================================================================
// CONSTRUCTION
//==============================================================================
//______________________________________________________________________________
/**
 * Destructor.
 */
BodyConstraint::~BodyConstraint()
{
}
//______________________________________________________________________________
/**
 * Constructor.
 */
BodyConstraint::BodyConstraint()
{
	clear();
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
 * @return Number of non-zero constraint directions.
 */
int BodyConstraint::
getNC()
{
	int i,nc;
	for(nc=i=0;i<3;i++) {
		nc += _pc[i].getNC();
	}
	return(nc);
}

//------------------------------------------------------------------------------
// ID
//------------------------------------------------------------------------------
//______________________________________________________________________________
/**
 * Set the body that this constraint is applied to.
 *
 * @param aBody Body that this constraint is applied to.
 */
void BodyConstraint::
setBody(AbstractBody *aBody)
{
	_body = aBody;
}
//______________________________________________________________________________
/**
 * Get the body that this constraint is applied to.
 *
 * @return Body that this constraint is applied to.
 */
AbstractBody* BodyConstraint::
getBody()
{
	return(_body);
}

//------------------------------------------------------------------------------
// POINT CONSTRAINTS
//------------------------------------------------------------------------------
//______________________________________________________________________________
/**
 * Get a pointer to a point constraint.
 *
 * @param aI Index of the desired point constraint.
 * @return Pointer to point constraint at index position aI.
 */
PointConstraint* BodyConstraint::
getPC(int aI)
{
	if(aI<0) return(NULL);
	if(aI>=3) return(NULL);
	return(&_pc[aI]);
}


//==============================================================================
// CONSTRAINT CONSTRUCTION
//==============================================================================
//______________________________________________________________________________
/**
 * Scan a list of point id's and set constraint values for any matching
 * point constraints.
 */
void BodyConstraint::
setValues(int aN,int aID[],double aV[][3])
{
	int i,j;
	for(i=0;i<aN;i++) {
		for(j=0;j<3;j++) {
			if(_pc[j].getID() == aID[i])  _pc[j].setValue(aV[i]);
		}
	}

}
//______________________________________________________________________________
/**
 * Construct the constraint directions for point 1.
 *
 * Two constraint directions are set for point 1 orthogonal to the vector
 * directed from point 0 to point 1.
 *
 * Note that it is assumed that the constraint directions for point 0 have
 * already been set properly.
 */
void BodyConstraint::
constructConstraintsForPoint1()
{
	// GET VECTOR FROM P0 to P1.
	double r1[3];
	Mtx::Subtract(1,3,_pc[1].getPoint(),_pc[0].getPoint(),r1);

	// FIND CONSTRAINT DIRECTION IN P0 THAT IS MOST ORTHOGONAL TO r
	double *cs =
		findMostOrthogonal(&_pc[0],r1);

	// CONSTRAINT DIRECTIONS
	double c0[3],c1[3];
	Mtx::CrossProduct(r1,cs,c0);
	Mtx::CrossProduct(c0,r1,c1);

	// SET AND NORMALIZE
	_pc[1].zeroConstraints();
	_pc[1].setC0(c0);
	_pc[1].setC1(c1);
	_pc[1].normalizeConstraints();
}
//______________________________________________________________________________
/**
 * Construct the constraint directions for point 2.
 *
 * One constraint direction is set for point 2 orthogonal to the vector
 * directed from point 0 to point 1 and the vector from point 1 to point 2.
 *
 * Note that it is assumed that the constraint directions for point 0 
 * and Point 1 have already been set properly.
 */
void BodyConstraint::
constructConstraintsForPoint2()
{
	// GET VECTOR FROM P0 to P1
	double r01[3],r12[3];
	Mtx::Subtract(1,3,_pc[1].getPoint(),_pc[0].getPoint(),r01);
	Mtx::Subtract(1,3,_pc[2].getPoint(),_pc[1].getPoint(),r12);

	// CONSTRAINT DIRECTIONS
	double c0[3];
	Mtx::CrossProduct(r01,r12,c0);
	Mtx::Normalize(3,c0,c0);

	// SET AND NORMALIZE
	_pc[2].zeroConstraints();
	_pc[2].setC0(c0);
}
//______________________________________________________________________________
/**
 * Find the constraint direction in the provided point constraint that is most
 * orthogonal to the provided vector.
 */
double* BodyConstraint::
findMostOrthogonal(PointConstraint *aPC,double aV[3])
{
	// CORSS PRODUCTS
	double theta0 = fabs( Mtx::Angle(aV,aPC->getC0()) );
	double theta1 = fabs( Mtx::Angle(aV,aPC->getC1()) );
	double theta2 = fabs( Mtx::Angle(aV,aPC->getC2()) );

	// CLOSSEST TO PI/4
	double pi4 = rdMath::PI/4.0;
	double d0 = fabs( theta0 - pi4 );
	double d1 = fabs( theta1 - pi4 );
	double d2 = fabs( theta2 - pi4 );

	// COMPARE
	double *cs = aPC->getC0();
	if(d1<d0) {
		cs = aPC->getC1();
		if(d2<d1) {
			cs = aPC->getC2();
			return(cs);
		}
	} else if(d2<d0) {
		cs = aPC->getC2();
		return(cs);
	}

	return(cs);
}


//==============================================================================
// UTILITY
//==============================================================================
//______________________________________________________________________________
/**
 * Clear all point constraints.
 *
 * @see PointConstraint#clear();
 */
void BodyConstraint::
clear()
{
	_pc[0].clear();
	_pc[1].clear();
	_pc[2].clear();
}
//______________________________________________________________________________
/**
 * Clear all constraint values.
 */
void BodyConstraint::
clearValues()
{
	_pc[0].setValue(0.0,0.0,0.0);
	_pc[1].setValue(0.0,0.0,0.0);
	_pc[2].setValue(0.0,0.0,0.0);
}

