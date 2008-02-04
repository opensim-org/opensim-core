// StepFunction.cpp
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


// C++ INCLUDES
#include "StepFunction.h"
#include "rdMath.h"
#include "PropertyInt.h"
#include "PropertyDbl.h"
#include "PropertyDblArray.h"
#include "SimmMacros.h"



using namespace OpenSim;
using namespace std;


//=============================================================================
// STATICS
//=============================================================================


//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
StepFunction::~StepFunction()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
StepFunction::StepFunction() :
	_x(_propX.getValueDblArray()),
	_y(_propY.getValueDblArray())
{
	setNull();
}
//_____________________________________________________________________________
/**
 */
StepFunction::StepFunction(int aN,const double *aX,const double *aY,
	const string &aName) :
	_x(_propX.getValueDblArray()),
	_y(_propY.getValueDblArray())
{
	setNull();

	// OBJECT TYPE AND NAME
	setName(aName);

	// NUMBER OF DATA POINTS
	if(aN < 2)
	{
		printf("StepFunction: ERROR- there must be 2 or more data points.\n");
		return;
	}

	// CHECK DATA
	if((aX==NULL)||(aY==NULL))
	{
		printf("StepFunction: ERROR- NULL arrays for data points encountered.\n");
		return;
	}

	// INDEPENDENT VALUES (KNOT SEQUENCE)
	_x.setSize(0);
	_x.append(aN,aX);

	_y.setSize(0);
	_y.append(aN,aY);
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 * All data members of the specified function are copied.
 *
 * @param aFunction StepFunction object to be copied.
 */
StepFunction::StepFunction(const StepFunction &aFunction) :
	Function(aFunction),
	_x(_propX.getValueDblArray()),
	_y(_propY.getValueDblArray())
{
	setEqual(aFunction);
}
//_____________________________________________________________________________
/**
 * Copy this object.
 *
 * @return Pointer to a copy of this object.
 */
Object* StepFunction::copy() const
{
	StepFunction *function = new StepFunction(*this);
	return(function);
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set all member variables to their NULL or default values.
 */
void StepFunction::setNull()
{
	setType("StepFunction");
	setupProperties();
}
//_____________________________________________________________________________
/**
 * Set up the serialized member variables.  This involves both generating
 * the properties and connecting them to the local pointers used to access
 * the serialized member variables.
 */
void StepFunction::setupProperties()
{
	// X- INDEPENDENT VARIABLES
	_propX.setName("x");
	Array<double> x(0.0);
	_propX.setValue(x);
	_propertySet.append( &_propX );

	// Y- DEPENDENT VARIABLES
	_propY.setName("y");
	Array<double> y(0.0);
	_propY.setValue(y);
	_propertySet.append( &_propY );
}
//_____________________________________________________________________________
/**
 * Set all member variables equal to the members of another object.
 * Note that this method is private.  It is only meant for copying the data
 * members defined in this class.  It does not, for example, make any changes
 * to data members of base classes.
 */
void StepFunction::setEqual(const StepFunction &aFunction)
{
	setNull();

	// CHECK ARRAY SIZES
	if(aFunction.getSize()<=0) return;

	// ALLOCATE ARRAYS
	_x = aFunction._x;
	_y = aFunction._y;
}
//_____________________________________________________________________________
/**
 * Initialize the function with X and Y values.
 *
 * @param aN the number of X and Y values
 * @param aXValues the X values
 * @param aYValues the Y values
 */
void StepFunction::init(int aN, const double *aXValues, const double *aYValues)
{
	StepFunction newStepFunction = StepFunction(aN, aXValues, aYValues);

	*this = newStepFunction;
}

//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 * Note that data members of the base class are also assigned.
 *
 * @return Reference to this object.
 */
StepFunction& StepFunction::operator=(const StepFunction &aFunction)
{
	// BASE CLASS
	Function::operator=(aFunction);

	// DATA
	setEqual(aFunction);

	return(*this);
}


//=============================================================================
// SET AND GET
//=============================================================================
//-----------------------------------------------------------------------------
// NUMBER OF DATA POINTS (N)
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get size or number of independent data points (or number of coefficients)
 * used to construct the function.
 *
 * @return Number of data points (or number of coefficients).
 */
int StepFunction::getSize() const
{
	return(_x.getSize());
}

//-----------------------------------------------------------------------------
// MIN AND MAX X
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the minimum value of the independent variable.
 *
 * @return Minimum value of the independent variable.
 */
double StepFunction::getMinX() const
{
	if(getSize()<=0) return(rdMath::NAN);
	return(_x.get(0));
}
//_____________________________________________________________________________
/**
 * Get the maximum value of the independent variable.
 *
 * @return Maximum value of the independent variable.
 */
double StepFunction::getMaxX() const
{
	if(getSize()<=0) return(rdMath::NAN);
	return(_x.getLast());
}

//-----------------------------------------------------------------------------
// X AND COEFFICIENTS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the array of independent variables used to construct the function.
 * For the number of independent variable data points use getN().
 *
 * @return Pointer to the independent variable data points.
 * @see getN();
 */
const Array<double>& StepFunction::getX() const
{
	return(_x);
}
//_____________________________________________________________________________
/**
 * Get the array of Y values for the function.
 * For the number of Y values use getNX().
 *
 * @return Pointer to the coefficients.
 * @see getCoefficients();
 */
const Array<double>& StepFunction::getY() const
{
	return(_y);
}
//_____________________________________________________________________________
/**
 * Get the array of independent variables used to construct the function.
 * For the number of independent variable data points use getN().
 *
 * @return Pointer to the independent variable data points.
 * @see getN();
 */
const double* StepFunction::getXValues() const
{
	return(&_x[0]);
}
//_____________________________________________________________________________
/**
 * Get the array of dependent variables used to construct the function.
 *
 * @return Pointer to the dependent variable data points.
 */
const double* StepFunction::getYValues() const
{
	return(&_y[0]);
}


//-----------------------------------------------------------------------------
// UPDATE FROM XML NODE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Update this object based on its XML node.
 */
void StepFunction::updateFromXMLNode()
{
	Function::updateFromXMLNode();
}	

//=============================================================================
// EVALUATION
//=============================================================================
//_____________________________________________________________________________
/**
 * Update the bounding box for this function.
 *
 * For an StepFunction, there is only one indepdendent variable x, so the 
 * minimum and maximum values of indepdent variables y and z is 0.0.
 *
 * When this method is called, the minimum and maximum x of the bounding box
 * is simply set to the minimum and maximum values of the x data points that
 * were used to construct the function, that is, min x = x[0] and
 * max x = x[getN()-1].
 *
 * @see Function
 */
void StepFunction::updateBoundingBox()
{
	setMinX(0.0);
	setMinY(0.0);
	setMinZ(0.0);
	setMaxX(0.0);
	setMaxY(0.0);
	setMaxZ(0.0);

	if(getSize()<=0) return;

	setMinX(_x.get(0));
	setMaxX(_x.getLast());
}

/**
 * Evaluates function or its (partial) derivatives
 */
double StepFunction::evaluate(int aDerivOrder, double aX, double aY, double aZ)
{
	if (aDerivOrder < 0)
		return rdMath::NAN;
	if (aDerivOrder > 0)
		return 0.0;

	int n = _x.getSize();

	if (aX < _x[0] || EQUAL_WITHIN_ERROR(aX,_x[0]))
		return _y[0];
   if (aX > _x[n-1] || EQUAL_WITHIN_ERROR(aX,_x[n-1]))
      return _y[n-1];

   // Do a binary search to find which two points the abscissa is between.
	int k, i = 0;
	int j = n;
	while (1)
	{
		k = (i+j)/2;
		if (aX < _x[k])
			j = k;
		else if (aX > _x[k+1])
			i = k;
		else
			break;
	}

	return _y[k];
}

/**
 * Evaluates total first derivative
 */
double StepFunction::
evaluateTotalFirstDerivative(double aX,double aDxdt)
{
	return 0.0;
}

/**
 * Evaluates total second derivative
 */
double StepFunction::
evaluateTotalSecondDerivative(double aX,double aDxdt,double aD2xdt2)
{
	return 0.0;
}

double StepFunction::getX(int aIndex) const
{
	if (aIndex >= 0 && aIndex < _x.getSize())
		return _x.get(aIndex);
	else {
		throw Exception("StepFunction::getX(): index out of bounds.");
		return 0.0;
	}
}

double StepFunction::getY(int aIndex) const
{
	if (aIndex >= 0 && aIndex < _y.getSize())
		return _y.get(aIndex);
	else {
		throw Exception("StepFunction::getY(): index out of bounds.");
		return 0.0;
	}
}

void StepFunction::setX(int aIndex, double aValue)
{
	if (aIndex >= 0 && aIndex < _x.getSize()) {
		_x[aIndex] = aValue;
	} else {
		throw Exception("StepFunction::setX(): index out of bounds.");
	}
}

void StepFunction::setY(int aIndex, double aValue)
{
	if (aIndex >= 0 && aIndex < _y.getSize()) {
		_y[aIndex] = aValue;
	} else {
		throw Exception("StepFunction::setY(): index out of bounds.");
	}
}

void StepFunction::scaleY(double aScaleFactor)
{
	for (int i = 0; i < _y.getSize(); i++)
		_y[i] *= aScaleFactor;
}

bool StepFunction::deletePoint(int aIndex)
{
	if (_x.getSize() > 1 && _y.getSize() > 1 &&
		 aIndex < _x.getSize() && aIndex < _y.getSize()) {
	   _x.remove(aIndex);
	   _y.remove(aIndex);
		return true;
	}

	return false;
}

void StepFunction::addPoint(double aX, double aY)
{
	for (int i=0; i<_x.getSize(); i++)
		if (_x[i] > aX)
			break;

	_x.insert(i, aX);
	_y.insert(i, aY);
}

Array<XYPoint>* StepFunction::renderAsLineSegments(double aStart, double aEnd)
{
	Array<XYPoint>* foo = new Array<XYPoint>(XYPoint());

	return foo;
}

Array<XYPoint>* StepFunction::renderAsLineSegments(int aIndex)
{
	if (aIndex < 0 || aIndex >= getNumberOfPoints() - 1)
		return NULL;

	Array<XYPoint>* xyPts = new Array<XYPoint>(XYPoint());

	xyPts->append(XYPoint(_x[aIndex], _y[aIndex]));
	xyPts->append(XYPoint(_x[aIndex+1], _y[aIndex]));
	xyPts->append(XYPoint(_x[aIndex+1], _y[aIndex+1]));

	return xyPts;
}
