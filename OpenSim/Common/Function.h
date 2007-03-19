#ifndef _Function_h_
#define _Function_h_
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
#include <stdlib.h>
#include <stdio.h>
#include <fstream>
#include "osimCommon.h"
#include "Object.h"
#include "PropertyDbl.h"


//=============================================================================
//=============================================================================
/**
 * An abstract class for representing a function.
 *
 * A function is a relation between independent variables and a dependent
 * value such that for any particular set of independent variables there is
 * only one unique dependent value.  Values of the function and its derivatives
 * are obtained by calling the evaluate() method.  The curve may or may not
 * be finite or diferentiable; the evaluate method returns values between
 * rdMath::MINUS_INFINITY and rdMath::PLUS_INFINITY, or it returns rdMath::NAN
 * (not a number) if the curve is not defined.
 * Currently, functions of up to 3 variables (x,y,z) are supported.
 *
 * @author Frank C. Anderson
 */
namespace OpenSim { 

class OSIMCOMMON_API Function : public Object
{
//=============================================================================
// DATA
//=============================================================================
protected:
	// PROPERTIES
	/** Minimum value of the x independent variable. */
	PropertyDbl _propMinX;
	/** Maximum value of the x independent variable. */
	PropertyDbl _propMaxX;
	/** Minimum value of the y independent variable. */
	PropertyDbl _propMinY;
	/** Maximum value of the y independent variable. */
	PropertyDbl _propMaxY;
	/** Minimum value of the z independent variable. */
	PropertyDbl _propMinZ;
	/** Maximum value of the z independent variable. */
	PropertyDbl _propMaxZ;

	// REFERENCES
	double &_minX;
	double &_maxX;
	double &_minY;
	double &_maxY;
	double &_minZ;
	double &_maxZ;

//=============================================================================
// METHODS
//=============================================================================
public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	Function();
	Function(const Function &aFunction);
	virtual ~Function();
	virtual Object* copy() const = 0;

	static Function* safeDownCast(Object* aObject) { return dynamic_cast<Function*>(aObject); }

private:
	void setNull();
	void setupProperties();
	void setEqual(const Function &aFunction);

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	Function& operator=(const Function &aFunction);
#endif
	//--------------------------------------------------------------------------
	// SET AND GET
	//--------------------------------------------------------------------------
public:
	void setMinX(double aMinX);
	double getMinX() const;
	void setMaxX(double aMaxX);
	double getMaxX() const;
	void setMinY(double aMinY);
	double getMinY() const;
	void setMaxY(double aMaxY);
	double getMaxY() const;
	void setMinZ(double aMinZ);
	double getMinZ() const;
	void setMaxZ(double aMaxZ);
	double getMaxZ() const;

	//--------------------------------------------------------------------------
	// EVALUATE
	//--------------------------------------------------------------------------
	virtual void updateBoundingBox() = 0;
	virtual int getNumberOfPoints() const = 0;
	virtual double
		evaluate(int aDerivOrder,double aX=0.0,double aY=0.0,double aZ=0.0) = 0;
	virtual void scaleY(double aScaleFactor) = 0;

//=============================================================================
};	// END class Function

}; //namespace
//=============================================================================
//=============================================================================

#endif  // __Function_h__
