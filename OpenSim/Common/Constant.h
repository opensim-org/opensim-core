#ifndef __Constant_h__
#define __Constant_h__

// Constant.h
// Author: Peter Loan
/*
 * Copyright (c) 2006, Stanford University. All rights reserved. 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including 
 * without limitation the rights to use, copy, modify, merge, publish, 
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included 
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */


// INCLUDES
#include <string>
#include <OpenSim/Tools/Function.h>
#include <OpenSim/Tools/PropertyDbl.h>

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class for representing a constant value.
 *
 * This class inherits from Function and so can be used as input to
 * any class requiring an rdFuction as input.
 *
 * @author Peter Loan
 * @version 1.0
 */
class RDTOOLS_API Constant : public Function
{
//=============================================================================
// MEMBER VARIABLES
//=============================================================================
protected:
	PropertyDbl _valueProp;
	double &_value;

private:

//=============================================================================
// METHODS
//=============================================================================
public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	Constant();
	Constant(int aN,const double *aTimes,const double *aValues,
		const std::string &aName="");
	Constant(const Constant &aSpline);
	virtual ~Constant();
	virtual Object* copy() const;

private:
	void setNull();
	void setupProperties();
	void copyData(const Constant &aConstant);

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	Constant& operator=(const Constant &aConstant);
#endif

	//--------------------------------------------------------------------------
	// SET AND GET
	//--------------------------------------------------------------------------
public:
	virtual int getNumberOfPoints() const { return 0; }
	void setValue(double aValue) { _value = aValue; }
	virtual void peteTest() const;

	//--------------------------------------------------------------------------
	// EVALUATION
	//--------------------------------------------------------------------------
	virtual void updateBoundingBox();
	virtual double	evaluate(int aDerivOrder, double aX=0.0, double aY=0.0, double aZ=0.0) { return _value; }
	double evaluate() { return _value; }
	virtual void scaleY(double aScaleFactor) { _value *= aScaleFactor; }

	virtual void updateFromXMLNode();

//=============================================================================
};	// END class Constant
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif  // __Constant_h__
