// Plane.cpp
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */

#include "rdMath.h"
#include "Mtx.h"
#include "Line.h"
#include "Plane.h"


//=============================================================================
// STATIC VARIABLES
//=============================================================================


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________


using namespace OpenSim;
/**
 * Destructor.
 */
Plane::~Plane()
{
}

//_____________________________________________________________________________
/**
 * Construct a plane from three points.
 *
 * @param aPX X component of a point through which the line passes.
 * @param aPY Y component of a point through which the line passes.
 * @param aPZ Z component of a point through which the line passes.
 * @param aP2X X component of a second point through which the line passes.
 * @param aP2Y Y component of a second point through which the line passes.
 * @param aP2Z Z component of a second point through which the line passes.
 * @param aP3X X component of a third point through which the line passes.
 * @param aP3Y Y component of a third point through which the line passes.
 * @param aP3Z Z component of a third point through which the line passes.
 * @param aNormal Unit vector normal to the plane.
 */
Plane::Plane(double aPX,double aPY,double aPZ,
					  double aP2X,double aP2Y,double aP2Z,
					  double aP3X,double aP3Y,double aP3Z)
{
	setNull();

	// TYPE
	setType("Plane");

	// COMPUTE THE NORMAL FROM THREE POINTS
	double normal[3];
	rdMath::ComputeNormal(aPX,aPY,aPZ,aP2X,aP2Y,aP2Z,aP3X,aP3Y,aP3Z,normal);

	// MEMBER VARIABLES
	setPoint(aPX,aPY,aPZ);
	setNormal(normal);
}


//_____________________________________________________________________________
/**
 * Construct a plane from a point and a normal.
 *
 * @param aPX X component of a point through which the line passes.
 * @param aPY Y component of a point through which the line passes.
 * @param aPZ Z component of a point through which the line passes.
 * @param aNormal Unit vector normal to the plane.
 */
Plane::Plane(double aPX,double aPY,double aPZ,const double aNormal[3])
{
	setNull();

	// TYPE
	setType("Plane");

	// MEMBER VARIABLES
	setPoint(aPX,aPY,aPZ);
	setNormal(aNormal);
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL values for all member variables.
 */
void Plane::
setNull()
{
	// POINT
	_point[0] = _point[1] = _point[2] = 0.0;

	// NORMAL
	_normal[0] = 0.0;
	_normal[1] = 0.0;
	_normal[2] = 0.0;
}



//=============================================================================
// SET / GET
//=============================================================================
//-----------------------------------------------------------------------------
// POINT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set a point through which the plane passes.
 *
 * @param aPX X component of a point through which the line passes.
 * @param aPY Y component of a point through which the line passes.
 * @param aPZ Z component of a point through which the line passes.
 * @param aPoint Point through which the plane passes.
 */
void Plane::
setPoint(double aPX,double aPY,double aPZ)
{
	// POINT
	_point[0] = aPX;
	_point[1] = aPY;
	_point[2] = aPZ;
}
//_____________________________________________________________________________
/**
 * Get a point through which the plane passes.
 *
 * @param aPoint Point through which the plane passes.
 */
void Plane::
getPoint(double rPoint[3]) const
{
	if(rPoint==NULL) return;

	// POINT
	rPoint[0] = _point[0];
	rPoint[1] = _point[1];
	rPoint[2] = _point[2];
}

//-----------------------------------------------------------------------------
// NORMAL
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the normal of the plane.
 *
 * @param aNormal Normal of the plane.  If aNormal is normalized so that it
 * is assured to be a unit vector.
 */
void Plane::
setNormal(const double aNormal[3])
{
	if(aNormal==NULL) return;

	// DIRECTION
	_normal[0] = aNormal[0];
	_normal[1] = aNormal[1];
	_normal[2] = aNormal[2];

	// NORMALIZED
	double mag = Mtx::Normalize(3,_normal,_normal);

	// ERROR CHECK
	if(mag<rdMath::ZERO) {
		printf("Plane.setNormal: WARN- plane normal is zero or very small.\n");
		printf("\tPlane normal may be ill defined.\n");
	}
}
//_____________________________________________________________________________
/**
 * Get the normal of the plane.
 *
 * @param aNormal Normal of the plane.
 */
void Plane::
getNormal(double rNormal[3]) const
{
	if(rNormal==NULL) return;

	// POINT
	rNormal[0] = _normal[0];
	rNormal[1] = _normal[1];
	rNormal[2] = _normal[2];
}


//=============================================================================
// ANALYTIC GEOMETRY
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute the intersection of a line with the plane.
 * The line can either intersect the plane at a point, not intersect the
 * plane at all, or be coincident with the plane.
 *
 * @param aLine Line.
 * @param aPlane Plane.
 * @param rPoint Point of intersection.  rPoint is set to the "zero" point of
 * the line in the event that line and plane are coincident; it is unchanged
 * if the line and plane do not intersect.
 * @return -1 if the line and plane do not intersect; 0 if the line and plane
 * intersect at a point; 1 if the line is coincident with the plane.
 */
int Plane::
computeIntersection(const Line *aLine,double rPoint[3]) const
{
	return(rdMath::ComputeIntersection(aLine,this,rPoint));
}

