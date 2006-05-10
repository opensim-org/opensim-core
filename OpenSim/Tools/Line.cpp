// Line.cpp
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


#include "rdMath.h"
#include "Mtx.h"
#include "Line.h"


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
Line::~Line()
{
}

//_____________________________________________________________________________
/**
 * Construct a line from two points (P and P2).
 *
 * Point P is kept as the point through which the line passes, and P2 is used
 * to compute the direction of the line.  The direction of the line
 * points from P to P2.
 *
 * @param aPX X component of a point through which the line passes.
 * @param aPY Y component of a point through which the line passes.
 * @param aPZ Z component of a point through which the line passes.
 * @param aP2X X component of a second point through which the line passes.
 * @param aP2Y Y component of a second point through which the line passes.
 * @param aP2Z Z component of a second point through which the line passes.
 * @param aDirection Direction of the line specified as a unit vector.
 */
Line::Line(double aPX,double aPY,double aPZ,
					double aP2X,double aP2Y,double aP2Z)
{
	setNull();

	// TYPE
	setType("Line");

	// COMPUTE DIRECTION
	double direction[3];
	direction[0] = aP2X - aPX;
	direction[1] = aP2Y - aPY;
	direction[2] = aP2Z - aPZ;

	// MEMBER VARIABLES
	setPoint(aPX,aPY,aPZ);
	setDirection(_direction);
}

//_____________________________________________________________________________
/**
 * Construct a line from a point and a direction.
 *
 * @param aPX X component of a point through which the line passes.
 * @param aPY Y component of a point through which the line passes.
 * @param aPZ Z component of a point through which the line passes.
 * @param aDirection Direction of the line specified as a unit vector.
 */
Line::Line(double aPX,double aPY,double aPZ,const double aDirection[3])
{
	setNull();

	// TYPE
	setType("Line");

	// MEMBER VARIABLES
	setPoint(aPX,aPY,aPZ);
	setDirection(aDirection);
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL values for all member variables.
 */
void Line::
setNull()
{
	// POINT
	_point[0] = _point[1] = _point[2] = 0.0;

	// DIRECTION
	_direction[0] = 1.0;
	_direction[1] = 0.0;
	_direction[2] = 0.0;
}


//=============================================================================
// SET / GET
//=============================================================================
//-----------------------------------------------------------------------------
// POINT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the point that the line passes through.
 *
 * @param aPX X component of a point through which the line passes.
 * @param aPY Y component of a point through which the line passes.
 * @param aPZ Z component of a point through which the line passes.
 */
void Line::
setPoint(double aPX,double aPY,double aPZ)
{
	_point[0] = aPX;
	_point[1] = aPY;
	_point[2] = aPZ;
}
//_____________________________________________________________________________
/**
 * Get the point that the line passes through.
 *
 * @param rPoint Point that the line passes through.
 */
void Line::
getPoint(double rPoint[3]) const
{
	if(rPoint==NULL) return;

	// POINT
	rPoint[0] = _point[0];
	rPoint[1] = _point[1];
	rPoint[2] = _point[2];
}

//-----------------------------------------------------------------------------
// DIRECTION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the direction of the line.
 *
 * @param aDirection Direction of the line.
 */
void Line::
setDirection(const double aDirection[3])
{
	if(aDirection==NULL) return;

	// POINT
	_direction[0] = aDirection[0];
	_direction[1] = aDirection[1];
	_direction[2] = aDirection[2];

	// NORMALIZED
	double mag = Mtx::Normalize(3,_direction,_direction);

	// ERROR CHECK
	if(mag<rdMath::ZERO) {
		printf("Plane.setNormal: WARN- plane normal is zero or very small.\n");
		printf("\tPlane normal may be ill defined.\n");
	}
}
//_____________________________________________________________________________
/**
 * Get the direction of the line.
 *
 * @param aDirection Direction of the line.
 */
void Line::
getDirection(double rDirection[3]) const
{
	if(rDirection==NULL) return;

	// POINT
	rDirection[0] = _direction[0];
	rDirection[1] = _direction[1];
	rDirection[2] = _direction[2];
}


//=============================================================================
// ANALYTIC GEOMETRY
//=============================================================================
//-----------------------------------------------------------------------------
// EVALUATE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Evaluate the line at a specified distance away from the specified
 * point of the line:
 *
 *    value = point + distance * direction
 *
 * @param aDistance Distance away from the specified point of this line
 * (see setPoint()).  aDistance may be positive or negative.
 * @param aValue Point value of the line at distance aDistance.
 */
void Line::
evaluate(double aDistance,double aValue[3]) const
{
	if(aValue==NULL) return;
	Mtx::Multiply(1,3,_direction,aDistance,aValue);
	Mtx::Add(1,3,_point,aValue,aValue);
}
