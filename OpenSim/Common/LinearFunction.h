#ifndef _LinearFunction_h_
#define _LinearFunction_h_

// LinearFunction.h
// Author: Peter Loan
/*
 * Copyright (c) 2007, Stanford University. All rights reserved. 
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
#include "osimCommonDLL.h"
#include <string>
#include "Array.h"
#include "PropertyInt.h"
#include "PropertyDbl.h"
#include "PropertyDblArray.h"
#include "Function.h"


//=============================================================================
//=============================================================================
/**
 * A class implementing a linear function.
 *
 * This class inherits from Function and so can be used as input to
 * any class requiring a Function as input.
 *
 * @author Peter Loan
 * @version 1.0
 */
namespace OpenSim { 

class OSIMCOMMON_API LinearFunction : public Function
{
//=============================================================================
// MEMBER VARIABLES
//=============================================================================
protected:
	// PROPERTIES
	/** Array of values for the independent variables (i.e., the knot
	sequence).  This array must be monotonically increasing. */
	PropertyDblArray _propX;
	Array<double> &_x;

	/** Y values. */
	PropertyDblArray _propY;
	Array<double> &_y;

private:
	Array<double> _b;

//=============================================================================
// METHODS
//=============================================================================
public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	LinearFunction();
	LinearFunction(int aN,const double *aTimes,const double *aValues,
		const std::string &aName="");
	LinearFunction(const LinearFunction &aFunction);
	virtual ~LinearFunction();
	virtual Object* copy() const;
	virtual void init(int aN, const double *aXValues, const double *aYValues);

private:
	void setNull();
	void setupProperties();
	void setEqual(const LinearFunction &aFunction);

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
	LinearFunction& operator=(const LinearFunction &aFunction);

	//--------------------------------------------------------------------------
	// SET AND GET
	//--------------------------------------------------------------------------
public:
	int getSize() const;
	double getMinX() const;
	double getMaxX() const;
	const Array<double>& getX() const;
	const Array<double>& getY() const;
	virtual const double* getXValues() const;
	virtual const double* getYValues() const;
	virtual int getNumberOfPoints() const { return _x.getSize(); }
	virtual double getX(int aIndex) const;
	virtual double getY(int aIndex) const;
	virtual double getZ(int aIndex) const { return 0.0; }
	virtual void setX(int aIndex, double aValue);
	virtual void setY(int aIndex, double aValue);
	virtual void scaleY(double aScaleFactor);
	virtual void deletePoint(int aIndex);
	virtual void addPoint(double aX, double aY);
	virtual Array<XYPoint>* renderAsLineSegments(double aStart, double aEnd);
	virtual Array<XYPoint>* renderAsLineSegments(int aIndex);

	//--------------------------------------------------------------------------
	// EVALUATION
	//--------------------------------------------------------------------------
	virtual void updateBoundingBox();
	virtual double	evaluate(int aDerivOrder, double aX=0.0, double aY=0.0, double aZ=0.0);
	virtual double evaluateTotalFirstDerivative(double aX,double aDxdt);
	virtual double evaluateTotalSecondDerivative(double aX,double aDxdt,double aD2xdt2);

	virtual void updateFromXMLNode();

private:
   void calcCoefficients();
	double evaluate(double aX, double velocity, double acceleration, int aDerivOrder);

	OPENSIM_DECLARE_DERIVED(LinearFunction, Function)

//=============================================================================
};	// END class LinearFunction

}; //namespace
//=============================================================================
//=============================================================================

#endif  // __LinearFunction_h__
