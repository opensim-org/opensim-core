#ifndef _NatCubicSpline_h_
#define _NatCubicSpline_h_

// NatCubicSpline.h
// Author: Peter Loan
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


// INCLUDES
#include "rdTools.h"
#include <string>
#include "Array.h"
#include "PropertyInt.h"
#include "PropertyDbl.h"
#include "PropertyDblArray.h"
#include "Function.h"


//=============================================================================
//=============================================================================
/**
 * A class implementing a smooth function with a natural cubic spline.
 *
 * This class inherits from Function and so can be used as input to
 * any class requiring an rdFuction as input.
 *
 * @author Peter Loan
 * @version 1.0
 */
namespace OpenSim { 

class RDTOOLS_API NatCubicSpline : public Function
{
//=============================================================================
// MEMBER VARIABLES
//=============================================================================
protected:
	// PROPERTIES
	/** Array of values for the independent variables (i.e., the spline knot
	sequence).  This array must be monotonically increasing. */
	PropertyDblArray _propX;
	Array<double> &_x;

	/** Y values. */
	PropertyDblArray _propY;
	Array<double> &_y;

private:
	Array<double> _b;
	Array<double> _c;
	Array<double> _d;

//=============================================================================
// METHODS
//=============================================================================
public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	NatCubicSpline();
	NatCubicSpline(int aN,const double *aTimes,const double *aValues,
		const std::string &aName="");
	NatCubicSpline(const NatCubicSpline &aSpline);
	virtual ~NatCubicSpline();
	virtual Object* copy() const;

private:
	void setNull();
	void setupProperties();
	void setEqual(const NatCubicSpline &aSpline);

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
	NatCubicSpline& operator=(const NatCubicSpline &aSpline);

	//--------------------------------------------------------------------------
	// SET AND GET
	//--------------------------------------------------------------------------
public:
	int getSize() const;
	double getMinX() const;
	double getMaxX() const;
	const Array<double>& getX() const;
	const Array<double>& getY() const;
	virtual int getNumberOfPoints() const { return _x.getSize(); }
	virtual void scaleY(double aScaleFactor);

	void peteTest() const;

	//--------------------------------------------------------------------------
	// EVALUATION
	//--------------------------------------------------------------------------
	virtual void updateBoundingBox();
	virtual double	evaluate(int aDerivOrder, double aX=0.0, double aY=0.0, double aZ=0.0);
	double evaluate(double aX, double velocity, double acceleration, int aDerivOrder);

	virtual void updateFromXMLNode();

private:
	void calcCoefficients();

	OpenSim_DERIVED(NatCubicSpline, Function)

//=============================================================================
};	// END class NatCubicSpline

}; //namespace
//=============================================================================
//=============================================================================

#endif  // __NatCubicSpline_h__
