#ifndef _Plane_h_
#define _Plane_h_
// Plane.h
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
#include "osimCommon.h"
#include "Object.h"

namespace OpenSim { 

class Line;

//=============================================================================
//=============================================================================
/**
 * A class for representing a plane.
 */
class OSIMCOMMON_API Plane : public Object
{
//=============================================================================
// DATA
//=============================================================================
private:
	/** Point through which the plane passes. */
	double _point[3];
	/** Unit vector normal to the plane. */
	double _normal[3];

//=============================================================================
// METHODS
//=============================================================================
public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	Plane(double aPX,double aPY,double aPZ,
		double aP2X,double aP2Y,double aP2Z,
		double aP3X,double aP3Y,double aP3Z);
	Plane(double aPX,double aPY,double aPZ,const double aNormal[3]);
	virtual ~Plane();
private:
	void setNull();

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
public:
	void setPoint(double aPX,double aPY,double aPZ);
	void getPoint(double rPoint[3]) const;
	void setNormal(const double aNormal[3]);
	void getNormal(double rNormal[3]) const;

	//--------------------------------------------------------------------------
	// ANALYTIC GEOMETRY
	//--------------------------------------------------------------------------
	int computeIntersection(const Line *aLine,double rPoint[3]) const;

//=============================================================================
};	// END class Plane

}; //namespace
//=============================================================================
//=============================================================================

#endif  // __Plane_h__
