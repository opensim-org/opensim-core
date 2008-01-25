// NatCubicSpline.cpp
// Author: Peter Loan
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


// C++ INCLUDES
#include "NatCubicSpline.h"
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
NatCubicSpline::~NatCubicSpline()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
NatCubicSpline::NatCubicSpline() :
	_x(_propX.getValueDblArray()),
	_y(_propY.getValueDblArray()),
	_b(0.0), _c(0.0), _d(0.0)
{
	setNull();
}
//_____________________________________________________________________________
/**
 */
NatCubicSpline::NatCubicSpline(int aN,const double *aX,const double *aY,
	const string &aName) :
	_x(_propX.getValueDblArray()),
	_y(_propY.getValueDblArray()),
	_b(0.0), _c(0.0), _d(0.0)
{
	setNull();

	// OBJECT TYPE AND NAME
	setName(aName);

	// NUMBER OF DATA POINTS
	if(aN < 2)
	{
		printf("NatCubicSpline: ERROR- there must be 2 or more data points.\n");
		return;
	}

	// CHECK DATA
	if((aX==NULL)||(aY==NULL))
	{
		printf("NatCubicSpline: ERROR- NULL arrays for data points encountered.\n");
		return;
	}

	// INDEPENDENT VALUES (KNOT SEQUENCE)
	_x.setSize(0);
	_x.append(aN,aX);

	_y.setSize(0);
	_y.append(aN,aY);

	// FIT THE SPLINE
	calcCoefficients();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 * All data members of the specified spline are copied.
 *
 * @param aSpline NatCubicSpline object to be copied.
 */
NatCubicSpline::NatCubicSpline(const NatCubicSpline &aSpline) :
	Function(aSpline),
	_x(_propX.getValueDblArray()),
	_y(_propY.getValueDblArray()),
	_b(0.0), _c(0.0), _d(0.0)
{
	setEqual(aSpline);
}
//_____________________________________________________________________________
/**
 * Copy this object.
 *
 * @return Pointer to a copy of this object.
 */
Object* NatCubicSpline::copy() const
{
	NatCubicSpline *spline = new NatCubicSpline(*this);
	return(spline);
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set all member variables to their NULL or default values.
 */
void NatCubicSpline::setNull()
{
	setType("NatCubicSpline");
	setupProperties();
}
//_____________________________________________________________________________
/**
 * Set up the serialized member variables.  This involves both generating
 * the properties and connecting them to the local pointers used to access
 * the serialized member variables.
 */
void NatCubicSpline::setupProperties()
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
void NatCubicSpline::setEqual(const NatCubicSpline &aSpline)
{
	setNull();

	// CHECK ARRAY SIZES
	if(aSpline.getSize()<=0) return;

	// ALLOCATE ARRAYS
	_x = aSpline._x;
	_y = aSpline._y;
	_b = aSpline._b;
	_c = aSpline._c;
	_d = aSpline._d;
}
//_____________________________________________________________________________
/**
 * Initialize the spline with X and Y values.
 *
 * @param aN the number of X and Y values
 * @param aXValues the X values
 * @param aYValues the Y values
 */
void NatCubicSpline::init(int aN, const double *aXValues, const double *aYValues)
{
	NatCubicSpline newSpline = NatCubicSpline(aN, aXValues, aYValues);

	*this = newSpline;
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
NatCubicSpline& NatCubicSpline::operator=(const NatCubicSpline &aSpline)
{
	// BASE CLASS
	Function::operator=(aSpline);

	// DATA
	setEqual(aSpline);

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
 * used to construct the spline.
 *
 * @return Number of data points (or number of coefficients).
 */
int NatCubicSpline::getSize() const
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
double NatCubicSpline::getMinX() const
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
double NatCubicSpline::getMaxX() const
{
	if(getSize()<=0) return(rdMath::NAN);
	return(_x.getLast());
}

//-----------------------------------------------------------------------------
// X AND COEFFICIENTS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the array of independent variables used to construct the spline.
 * For the number of independent variable data points use getN().
 *
 * @return Pointer to the independent variable data points.
 * @see getN();
 */
const Array<double>& NatCubicSpline::getX() const
{
	return(_x);
}
//_____________________________________________________________________________
/**
 * Get the array of Y values for the spline.
 * For the number of Y values use getNX().
 *
 * @return Pointer to the coefficients.
 * @see getCoefficients();
 */
const Array<double>& NatCubicSpline::getY() const
{
	return(_y);
}
//_____________________________________________________________________________
/**
 * Get the array of independent variables used to construct the spline.
 * For the number of independent variable data points use getN().
 *
 * @return Pointer to the independent variable data points.
 * @see getN();
 */
const double* NatCubicSpline::getXValues() const
{
	return(&_x[0]);
}
//_____________________________________________________________________________
/**
 * Get the array of dependent variables used to construct the spline.
 *
 * @return Pointer to the independent variable data points.
 */
const double* NatCubicSpline::getYValues() const
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
void NatCubicSpline::updateFromXMLNode()
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
 * For an NatCubicSpline, there is only one indepdendent variable x, so the 
 * minimum and maximum values of indepdent variables y and z is 0.0.
 *
 * When this method is called, the minimum and maximum x of the bounding box
 * is simply set to the minimum and maximum values of the x data points that
 * were used to construct the spline, that is, min x = x[0] and
 * max x = x[getN()-1].
 *
 * @see Function
 */
void NatCubicSpline::updateBoundingBox()
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

void NatCubicSpline::calcCoefficients()
{
   int n = _x.getSize();
	int nm1, nm2, i, j;
   double t;

   if (n < 2)
      return;

   _b.setSize(n);
   _c.setSize(n);
   _d.setSize(n);

   if (n == 2)
   {
      t = MAX(TINY_NUMBER,_x[1]-_x[0]);
      _b[0] = _b[1] = (_y[1]-_y[0])/t;
      _c[0] = _c[1] = 0.0;
      _d[0] = _d[1] = 0.0;
      return;
   }

   nm1 = n - 1;
   nm2 = n - 2;

   /* Set up tridiagonal system:
    * b = diagonal, d = offdiagonal, c = right-hand side
    */

   _d[0] = MAX(TINY_NUMBER,_x[1] - _x[0]);
   _c[1] = (_y[1]-_y[0])/_d[0];
   for (i=1; i<nm1; i++)
   {
      _d[i] = MAX(TINY_NUMBER,_x[i+1] - _x[i]);
      _b[i] = 2.0*(_d[i-1]+_d[i]);
      _c[i+1] = (_y[i+1]-_y[i])/_d[i];
      _c[i] = _c[i+1] - _c[i];
   }

   /* End conditions. Third derivatives at x[0] and x[n-1]
    * are obtained from divided differences.
    */

   _b[0] = -_d[0];
   _b[nm1] = -_d[nm2];
   _c[0] = 0.0;
   _c[nm1] = 0.0;

   if (n > 3)
   {
      double d1, d2, d3, d20, d30, d31;

      d31 = MAX(TINY_NUMBER,_x[3] - _x[1]);
      d20 = MAX(TINY_NUMBER,_x[2] - _x[0]);
      d1 = MAX(TINY_NUMBER,_x[nm1]-_x[n-3]);
      d2 = MAX(TINY_NUMBER,_x[nm2]-_x[n-4]);
      d30 = MAX(TINY_NUMBER,_x[3] - _x[0]);
      d3 = MAX(TINY_NUMBER,_x[nm1]-_x[n-4]);
      _c[0] = _c[2]/d31 - _c[1]/d20;
      _c[nm1] = _c[nm2]/d1 - _c[n-3]/d2;
      _c[0] = _c[0]*_d[0]*_d[0]/d30;
      _c[nm1] = -_c[nm1]*_d[nm2]*_d[nm2]/d3;
   }

   /* Forward elimination */

   for (i=1; i<n; i++)
   {
      t = _d[i-1]/_b[i-1];
      _b[i] -= t*_d[i-1];
      _c[i] -= t*_c[i-1];
   }

   /* Back substitution */

   _c[nm1] /= _b[nm1];
   for (j=0; j<nm1; j++)
   {
      i = nm2 - j;
      _c[i] = (_c[i]-_d[i]*_c[i+1])/_b[i];
   }

   /* compute polynomial coefficients */

   _b[nm1] = (_y[nm1]-_y[nm2])/_d[nm2] +
               _d[nm2]*(_c[nm2]+2.0*_c[nm1]);
   for (i=0; i<nm1; i++)
   {
      _b[i] = (_y[i+1]-_y[i])/_d[i] - _d[i]*(_c[i+1]+2.0*_c[i]);
      _d[i] = (_c[i+1]-_c[i])/_d[i];
      _c[i] *= 3.0;
   }
   _c[nm1] *= 3.0;
   _d[nm1] = _d[nm2];

}

/**
 * Evaluates function or its (partial) derivatives
 */
double NatCubicSpline::evaluate(int aDerivOrder, double aX, double aY, double aZ)
{
	// pass 0 for acceleration so that if we're computing the second derivative we get the proper partial derivative
	return evaluate(aX, 1.0, 0.0, aDerivOrder); 
}

/**
 * Evaluates total first derivative
 */
double NatCubicSpline::
evaluateTotalFirstDerivative(double aX,double aDxdt)
{
	return evaluate(aX, aDxdt, 0.0, 1);
}

/**
 * Evaluates total second derivative
 */
double NatCubicSpline::
evaluateTotalSecondDerivative(double aX,double aDxdt,double aD2xdt2)
{
	return evaluate(aX, aDxdt, aD2xdt2, 2);
}

//_____________________________________________________________________________
/**
 * Evaluate this function or a total derivative of this function given a set of
 * independent variables.  Only splines of one dimension are supported by
 * this class, so values of independent variables y and z are ignored.
 *
 */
double NatCubicSpline::evaluate(double aX, double velocity,
												double acceleration, int aDerivOrder)
{
	// NOT A NUMBER
	if(!_y.getSize()) return(rdMath::NAN);
	if(!_b.getSize()) return(rdMath::NAN);
	if(!_c.getSize()) return(rdMath::NAN);
	if(!_d.getSize()) return(rdMath::NAN);
	if(aDerivOrder<0) return(rdMath::NAN);

   int i, j, k;
   double dx;

	int n = _x.getSize();

   /* Check if the abscissa is out of range of the function. If it is,
    * then use the slope of the function at the appropriate end point to
    * extrapolate. You do this rather than printing an error because the
    * assumption is that this will only occur in relatively harmless
    * situations (like a motion file that contains an out-of-range coordinate
    * value). The rest of the SIMM code has many checks to clamp a coordinate
    * value within its range of motion, so if you make it to this function
    * and the coordinate is still out of range, deal with it quietly.
    */

   if (aX < _x[0])
   {
      if (aDerivOrder == 0)
         return _y[0] + (aX - _x[0])*_b[0];
      if (aDerivOrder == 1)
         return _b[0]*velocity;
      if (aDerivOrder == 2)
         return 0;
   }
   else if (aX > _x[n-1])
   {
      if (aDerivOrder == 0)
         return _y[n-1] + (aX - _x[n-1])*_b[n-1];
      if (aDerivOrder == 1)
         return _b[n-1]*velocity;
      if (aDerivOrder == 2)
         return 0;
   }

   /* Check to see if the abscissa is close to one of the end points
    * (the binary search method doesn't work well if you are at one of the
    * end points.
    */
   if (EQUAL_WITHIN_ERROR(aX,_x[0]))
   {
      if (aDerivOrder == 0)
         return _y[0];
      if (aDerivOrder == 1)
         return _b[0]*velocity;
      if (aDerivOrder == 2)
         return _b[0]*acceleration + 2.0*_c[0]*velocity*velocity;
   }
   else if (EQUAL_WITHIN_ERROR(aX,_x[n-1]))
   {
      if (aDerivOrder == 0)
         return _y[n-1];
      if (aDerivOrder == 1)
         return _b[n-1]*velocity;
      if (aDerivOrder == 2)
         return _b[n-1]*acceleration + 2.0*_c[n-1]*velocity*velocity;
   }

	if (n < 3)
	{
		/* If there are only 2 function points, then set k to zero
		 * (you've already checked to see if the abscissa is out of
		 * range or equal to one of the endpoints).
		 */
		k = 0;
	}
	else
	{
		/* Do a binary search to find which two points the abscissa is between. */
		i = 0;
		j = n;
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
	}

   dx = aX - _x[k];

   if (aDerivOrder == 0)
      return _y[k] + dx*(_b[k] + dx*(_c[k] + dx*_d[k]));

   if (aDerivOrder == 1)
      return (_b[k] + dx*(2.0*_c[k] + 3.0*dx*_d[k]))*velocity;

   if (aDerivOrder == 2)
      return (_b[k] + dx*(2.0*_c[k] + 3.0*dx*_d[k]))*acceleration +
	      (2.0*_c[k] + 6.0*dx*_d[k])*velocity*velocity;

   return rdMath::NAN;

}

double NatCubicSpline::getX(int aIndex) const
{
	if (aIndex >= 0 && aIndex < _x.getSize())
		return _x.get(aIndex);
	else {
		throw Exception("NatCubicSpline::getX(): index out of bounds.");
		return 0.0;
	}
}

double NatCubicSpline::getY(int aIndex) const
{
	if (aIndex >= 0 && aIndex < _y.getSize())
		return _y.get(aIndex);
	else {
		throw Exception("NatCubicSpline::getY(): index out of bounds.");
		return 0.0;
	}
}

void NatCubicSpline::setX(int aIndex, double aValue)
{
	if (aIndex >= 0 && aIndex < _x.getSize()) {
		_x[aIndex] = aValue;
	   calcCoefficients();
	} else {
		throw Exception("NatCubicSpline::setX(): index out of bounds.");
	}
}

void NatCubicSpline::setY(int aIndex, double aValue)
{
	if (aIndex >= 0 && aIndex < _y.getSize()) {
		_y[aIndex] = aValue;
	   calcCoefficients();
	} else {
		throw Exception("NatCubicSpline::setY(): index out of bounds.");
	}
}

void NatCubicSpline::scaleY(double aScaleFactor)
{
	for (int i = 0; i < _y.getSize(); i++)
		_y[i] *= aScaleFactor;

	// Recalculate the coefficients
	calcCoefficients();
}

void NatCubicSpline::deletePoint(int aIndex)
{
	_x.remove(aIndex);
	_y.remove(aIndex);

	// Recalculate the coefficients
	calcCoefficients();
}

void NatCubicSpline::addPoint(double aX, double aY)
{
	for (int i=0; i<_x.getSize(); i++)
		if (_x[i] > aX)
			break;

	_x.insert(i, aX);
	_y.insert(i, aY);

	// Recalculate the slopes
	calcCoefficients();
}

Array<XYPoint>* NatCubicSpline::renderAsLineSegments(double aStart, double aEnd)
{
	Array<XYPoint>* foo = new Array<XYPoint>(XYPoint());

	return foo;
}

Array<XYPoint>* NatCubicSpline::renderAsLineSegments(int aIndex)
{
	if (aIndex < 0 || aIndex >= getNumberOfPoints() - 1)
		return NULL;

	Array<XYPoint>* xyPts = new Array<XYPoint>(XYPoint());

	int numSegs = 20;
	for (int i=0; i<numSegs; i++) {
		double x = _x[aIndex] + (double)i * (_x[aIndex + 1] - _x[aIndex]) / ((double)numSegs - 1.0);
		double y = evaluate(x, 0.0, 0.0, 0);
		xyPts->append(XYPoint(x, y));
	}

	return xyPts;
}
