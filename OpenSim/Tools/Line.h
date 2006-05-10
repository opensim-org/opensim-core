#ifndef _Line_h_
#define _Line_h_
// Line.h
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
#include "rdTools.h"
#include "Object.h"

//=============================================================================
//=============================================================================
/**
 * A class for representing a straight line.
 */
namespace OpenSim { 

class RDTOOLS_API Line : public Object
{
//=============================================================================
// DATA
//=============================================================================
private:
	/** Point through which the line passes. */
	double _point[3];
	/** Unit vector specifying the direction of the line. */
	double _direction[3];

//=============================================================================
// METHODS
//=============================================================================
public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	Line(double aP0X,double aP0Y,double aP0Z,
		double aP2X,double aP2Y,double aP2Z);
	Line(double aPX,double aPY,double aPZ,const double aDirection[3]);
	virtual ~Line();
private:
	void setNull();

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
public:
	void setPoint(double aPX,double aPY,double aPZ);
	void getPoint(double rPoint[3]) const;
	void setDirection(const double aDirection[3]);
	void getDirection(double rDirection[3]) const;

	//--------------------------------------------------------------------------
	// ANALYTIC GEOMETRY
	//--------------------------------------------------------------------------
	void evaluate(double aDistance,double rLineValue[3]) const;

//=============================================================================
};	// END class Line

}; //namespace
//=============================================================================
//=============================================================================

#endif  // __Line_h__
