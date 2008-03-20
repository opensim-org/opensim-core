// BodyConstraint.cpp
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
#include <math.h>
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/Mtx.h>
#include "PointConstraint.h"
#include "BodyConstraint.h"


using namespace OpenSim;
using SimTK::Vec3;

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
			if(_pc[j].getID() == aID[i]){  
				Vec3 aVi = Vec3::getAs(aV[i]);
				_pc[j].setValue(aVi);
			}
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
	SimTK::Vec3 r1=_pc[1].getPoint()-_pc[0].getPoint();

	// FIND CONSTRAINT DIRECTION IN P0 THAT IS MOST ORTHOGONAL TO r
	SimTK::Vec3 cs = Vec3::getAs(findMostOrthogonal(&_pc[0],r1));

	// CONSTRAINT DIRECTIONS
	SimTK::Vec3 c0,c1;
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
	SimTK::Vec3 r01=_pc[1].getPoint()-_pc[0].getPoint();
	SimTK::Vec3 r12=_pc[2].getPoint()-_pc[1].getPoint();

	// CONSTRAINT DIRECTIONS
	SimTK::Vec3 c0;
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
findMostOrthogonal(PointConstraint *aPC,SimTK::Vec3& aV)
{
	// CORSS PRODUCTS
	double theta0 = fabs( Mtx::Angle(aV,aPC->getC0()) );
	double theta1 = fabs( Mtx::Angle(aV,aPC->getC1()) );
	double theta2 = fabs( Mtx::Angle(aV,aPC->getC2()) );

	// CLOSSEST TO PI/4
	double pi4 = SimTK_PI/4.0;
	double d0 = fabs( theta0 - pi4 );
	double d1 = fabs( theta1 - pi4 );
	double d2 = fabs( theta2 - pi4 );

	// COMPARE
	double *cs = &aPC->getC0()[0];
	if(d1<d0) {
		cs = &aPC->getC1()[0];
		if(d2<d1) {
			cs = &aPC->getC2()[0];
			return(cs);
		}
	} else if(d2<d0) {
		cs = &aPC->getC2()[0];
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

