// LinearFunction.cpp
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
#include "LinearFunction.h"
#include "Constant.h"
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
LinearFunction::~LinearFunction()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
LinearFunction::LinearFunction() :
	_x(_propX.getValueDblArray()),
	_y(_propY.getValueDblArray())
{
	setNull();
}
//_____________________________________________________________________________
/**
 */
LinearFunction::LinearFunction(int aN,const double *aX,const double *aY,
	const string &aName) :
	_x(_propX.getValueDblArray()),
	_y(_propY.getValueDblArray()),
   _b(0.0)
{
	setNull();

	// OBJECT TYPE AND NAME
	setName(aName);

	// NUMBER OF DATA POINTS
	if(aN < 2)
	{
		printf("LinearFunction: ERROR- there must be 2 or more data points.\n");
		return;
	}

	// CHECK DATA
	if((aX==NULL)||(aY==NULL))
	{
		printf("LinearFunction: ERROR- NULL arrays for data points encountered.\n");
		return;
	}

	// INDEPENDENT VALUES (KNOT SEQUENCE)
	_x.setSize(0);
	_x.append(aN,aX);

	_y.setSize(0);
	_y.append(aN,aY);

	// Calculate the slope coefficients
	calcCoefficients();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 * All data members of the specified function are copied.
 *
 * @param aFunction LinearFunction object to be copied.
 */
LinearFunction::LinearFunction(const LinearFunction &aFunction) :
	Function(aFunction),
	_x(_propX.getValueDblArray()),
	_y(_propY.getValueDblArray()),
   _b(0.0)
{
	setEqual(aFunction);
}
//_____________________________________________________________________________
/**
 * Copy this object.
 *
 * @return Pointer to a copy of this object.
 */
Object* LinearFunction::copy() const
{
	LinearFunction *function = new LinearFunction(*this);
	return(function);
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set all member variables to their NULL or default values.
 */
void LinearFunction::setNull()
{
	setType("LinearFunction");
	setupProperties();
}
//_____________________________________________________________________________
/**
 * Set up the serialized member variables.  This involves both generating
 * the properties and connecting them to the local pointers used to access
 * the serialized member variables.
 */
void LinearFunction::setupProperties()
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
void LinearFunction::setEqual(const LinearFunction &aFunction)
{
	setNull();

	// CHECK ARRAY SIZES
	if(aFunction.getSize()<=0) return;

	// ALLOCATE ARRAYS
	_x = aFunction._x;
	_y = aFunction._y;
	_b = aFunction._b;
}
//_____________________________________________________________________________
/**
 * Initialize the function with X and Y values.
 *
 * @param aN the number of X and Y values
 * @param aXValues the X values
 * @param aYValues the Y values
 */
void LinearFunction::init(Function* aFunction)
{
	if (aFunction == NULL)
		return;

	LinearFunction* lf = dynamic_cast<LinearFunction*>(aFunction);
	if (lf != NULL) {
		setEqual(*lf);
	} else if (aFunction->getNumberOfPoints() == 0) {
		// A LinearFunction must have at least 2 data points.
		// If aFunction is a Constant, use its Y value for both data points.
		// If it is not, make up two data points.
		double x[2] = {0.0, 1.0}, y[2];
		Constant* cons = dynamic_cast<Constant*>(aFunction);
		if (cons != NULL) {
			y[0] = y[1] = cons->evaluate();
		} else {
			y[0] = y[1] = 1.0;
		}
		*this = LinearFunction(2, x, y);
	} else if (aFunction->getNumberOfPoints() == 1) {
		double x[2], y[2];
		x[0] = aFunction->getXValues()[0];
		x[1] = x[0] + 1.0;
		y[0] = y[1] = aFunction->getYValues()[0];
		*this = LinearFunction(2, x, y);
	} else {
		*this = LinearFunction(aFunction->getNumberOfPoints(),
			aFunction->getXValues(), aFunction->getYValues());
	}
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
LinearFunction& LinearFunction::operator=(const LinearFunction &aFunction)
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
int LinearFunction::getSize() const
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
double LinearFunction::getMinX() const
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
double LinearFunction::getMaxX() const
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
const Array<double>& LinearFunction::getX() const
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
const Array<double>& LinearFunction::getY() const
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
const double* LinearFunction::getXValues() const
{
	return(&_x[0]);
}
//_____________________________________________________________________________
/**
 * Get the array of dependent variables used to construct the function.
 *
 * @return Pointer to the dependent variable data points.
 */
const double* LinearFunction::getYValues() const
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
void LinearFunction::updateFromXMLNode()
{
	Function::updateFromXMLNode();
	calcCoefficients();
}	

//=============================================================================
// EVALUATION
//=============================================================================
//_____________________________________________________________________________
/**
 * Update the bounding box for this function.
 *
 * For an LinearFunction, there is only one indepdendent variable x, so the 
 * minimum and maximum values of indepdent variables y and z is 0.0.
 *
 * When this method is called, the minimum and maximum x of the bounding box
 * is simply set to the minimum and maximum values of the x data points that
 * were used to construct the function, that is, min x = x[0] and
 * max x = x[getN()-1].
 *
 * @see Function
 */
void LinearFunction::updateBoundingBox()
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

void LinearFunction::calcCoefficients()
{
   int n = _x.getSize();

   if (n < 2)
      return;

   _b.setSize(n);

	for (int i=0; i<n-1; i++) {
		double range = MAX(TINY_NUMBER, _x[i+1] - _x[i]);
		_b[i] = (_y[i+1] - _y[i]) / range;
	}
	_b[n-1] = _b[n-2];
}

/**
 * Evaluates function or its (partial) derivatives
 */
double LinearFunction::evaluate(int aDerivOrder, double aX, double aY, double aZ)
{
	// pass 0 for acceleration so that if we're computing the second derivative we get the proper partial derivative
	return evaluate(aX, 1.0, 0.0, aDerivOrder); 
}

/**
 * Evaluates total first derivative
 */
double LinearFunction::
evaluateTotalFirstDerivative(double aX,double aDxdt)
{
	return evaluate(aX, aDxdt, 0.0, 1);
}

/**
 * Evaluates total second derivative
 */
double LinearFunction::
evaluateTotalSecondDerivative(double aX,double aDxdt,double aD2xdt2)
{
	return 0.0;
}

//_____________________________________________________________________________
/**
 * Evaluate this function or a total derivative of this function given a set of
 * independent variables.  Only functions of one dimension are supported by
 * this class, so values of independent variables y and z are ignored.
 *
 */
double LinearFunction::evaluate(double aX, double velocity,
										double acceleration, int aDerivOrder)
{
	if (aDerivOrder < 0)
		return rdMath::NAN;
	if (aDerivOrder > 1)
		return 0.0;

	int n = _x.getSize();

   if (aX < _x[0]) {
      if (aDerivOrder == 0)
         return _y[0] + (aX - _x[0]) * _b[0];
      if (aDerivOrder == 1)
         return _b[0] * velocity;
   } else if (aX > _x[n-1]) {
      if (aDerivOrder == 0)
         return _y[n-1] + (aX - _x[n-1]) * _b[n-1];
      if (aDerivOrder == 1)
         return _b[n-1] * velocity;
   }

   /* Check to see if the abscissa is close to one of the end points
    * (the binary search method doesn't work well if you are at one of the
    * end points.
    */
   if (EQUAL_WITHIN_ERROR(aX, _x[0])) {
      if (aDerivOrder == 0)
         return _y[0];
		if (aDerivOrder == 1)
         return _b[0] * velocity;
   } else if (EQUAL_WITHIN_ERROR(aX,_x[n-1])) {
      if (aDerivOrder == 0)
         return _y[n-1];
		if (aDerivOrder == 1)
         return _b[n-1] * velocity;
   }

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

   if (aDerivOrder == 0)
      return _y[k] + (aX - _x[k]) * _b[k];

   //derivOrder == 1
	return _b[k] * velocity;
}

double LinearFunction::getX(int aIndex) const
{
	if (aIndex >= 0 && aIndex < _x.getSize())
		return _x.get(aIndex);
	else {
		throw Exception("LinearFunction::getX(): index out of bounds.");
		return 0.0;
	}
}

double LinearFunction::getY(int aIndex) const
{
	if (aIndex >= 0 && aIndex < _y.getSize())
		return _y.get(aIndex);
	else {
		throw Exception("LinearFunction::getY(): index out of bounds.");
		return 0.0;
	}
}

void LinearFunction::setX(int aIndex, double aValue)
{
	if (aIndex >= 0 && aIndex < _x.getSize()) {
		_x[aIndex] = aValue;
		calcCoefficients();
	} else {
		throw Exception("LinearFunction::setX(): index out of bounds.");
	}
}

void LinearFunction::setY(int aIndex, double aValue)
{
	if (aIndex >= 0 && aIndex < _y.getSize()) {
		_y[aIndex] = aValue;
		calcCoefficients();
	} else {
		throw Exception("LinearFunction::setY(): index out of bounds.");
	}
}

void LinearFunction::scaleY(double aScaleFactor)
{
	for (int i = 0; i < _y.getSize(); i++)
		_y[i] *= aScaleFactor;

	// Recalculate the slopes
   calcCoefficients();
}

bool LinearFunction::deletePoint(int aIndex)
{
	if (_x.getSize() > 2 && _y.getSize() > 2 &&
		 aIndex < _x.getSize() && aIndex < _y.getSize()) {
	   _x.remove(aIndex);
	   _y.remove(aIndex);

	   // Recalculate the slopes
      calcCoefficients();
		return true;
	}

   return false;
}

bool LinearFunction::deletePoints(const Array<int>& indices)
{
	bool pointsDeleted = false;
	int numPointsLeft = _x.getSize() - indices.getSize();

	if (numPointsLeft >= 2) {
		// Assume the indices are sorted highest to lowest
		for (int i=0; i<indices.getSize(); i++) {
			int index = indices.get(i);
			if (index >= 0 && index < _x.getSize()) {
	         _x.remove(index);
	         _y.remove(index);
				pointsDeleted = true;
			}
		}
		if (pointsDeleted)
			calcCoefficients();
	}

   return pointsDeleted;
}

int LinearFunction::addPoint(double aX, double aY)
{
	int i=0;
	for (i=0; i<_x.getSize(); i++)
		if (_x[i] > aX)
			break;

	_x.insert(i, aX);
	_y.insert(i, aY);

	// Recalculate the slopes
	calcCoefficients();

	return i;
}

Array<XYPoint>* LinearFunction::renderAsLineSegments(double aStart, double aEnd)
{
	Array<XYPoint>* foo = new Array<XYPoint>(XYPoint());

	return foo;
}

Array<XYPoint>* LinearFunction::renderAsLineSegments(int aIndex)
{
	if (aIndex < 0 || aIndex >= getNumberOfPoints() - 1)
		return NULL;

	Array<XYPoint>* xyPts = new Array<XYPoint>(XYPoint());

	xyPts->append(XYPoint(_x[aIndex], _y[aIndex]));
	xyPts->append(XYPoint(_x[aIndex+1], _y[aIndex+1]));

	return xyPts;
}
