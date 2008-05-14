// GCVSpline.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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

/*  
 * Author:  Frank C. Anderson
 
 */


// C++ INCLUDES
#include "GCVSpline.h"
#include "Constant.h"
#include "rdMath.h"
#include "PropertyInt.h"
#include "PropertyDbl.h"
#include "PropertyDblArray.h"
#include "gcvspl.h"




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
GCVSpline::~GCVSpline()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
GCVSpline::
GCVSpline() :
	_halfOrder(_propHalfOrder.getValueInt()),
	_errorVariance(_propErrorVariance.getValueDbl()),
	_x(_propX.getValueDblArray()),
	_weights(_propWeights.getValueDblArray()),
	_coefficients(_propCoefficients.getValueDblArray()),
	_wk(_propWk.getValueDblArray()),
	_workEval(0.0),
	_y(0.0)
{
	setNull();
}
//_____________________________________________________________________________
/**
 * Construct a spline of a specified degree given arrays of paired data points
 * (x,f(x)). A name for the spline may be specified.
 *
 * @param aDegree Degree of the spline.  Only the following degrees
 * are supported: 1 = linear, 3 = cubic, 5 = quintic, and 7 = heptic.
 * @param aN Number of data points.
 * @param aX Array of independent values- should be aN long.
 * @param aF Array of function values- should be aN long.
 * @param aName Optional name of the spline.
 * @param aErrorVariance Estimate of the variance of the error in the data to
 * be fit.  If negative, the variance will be estimated.  If 0.0, the fit will
 * try to fit the data points exactly- no smoothing.  If positive, the fit
 * will be smoothed according to the specified variance. The larger the error
 * variance, the more the smoothing.  The smoothing parameter, p, in
 * Woltring (1986) is computed based on the error variance.
 */
GCVSpline::
GCVSpline(int aDegree,int aN,const double *aX,const double *aF,
	const string &aName,double aErrorVariance) :
	_halfOrder(_propHalfOrder.getValueInt()),
	_errorVariance(_propErrorVariance.getValueDbl()),
	_x(_propX.getValueDblArray()),
	_weights(_propWeights.getValueDblArray()),
	_coefficients(_propCoefficients.getValueDblArray()),
	_wk(_propWk.getValueDblArray()),
	_workEval(0.0),
	_y(0.0)
{
	setNull();

	// OBJECT TYPE AND NAME
	setName(aName);

	// DEGREE
	setDegree(aDegree);

	// NUMBER OF DATA POINTS
	if(aN < getOrder()) {
		printf("GCVSpline: ERROR- there must be %d or more data points.\n",
			getOrder());
		return;
	}

	// CHECK DATA
	if((aX==NULL)||(aF==NULL)) {
		printf("GCVSpline: ERROR- NULL arrays for data points encountered.\n");
		return;
	}

	// INDEPENDENT VALUES (KNOT SEQUENCE)
	_x.setSize(0);
	_x.append(aN,aX);

	// DEPENDENT VALUES
	_y.setSize(0);
	_y.append(aN,aF);

	// WEIGHTS
	int i;
	_weights.setSize(_x.getSize());
	for(i=0;i<_weights.getSize();i++) _weights[i] = 1.0;

	// WORK ARRAY FOR EVALUATION
	int nw = 2 * _halfOrder;
	_workEval.setSize(nw);

	// ALLOCATE COEFFICIENTS
	_coefficients.setSize(_x.getSize());

	// FIT THE SPLINE
	int ierr=0;
	_errorVariance = aErrorVariance;
	int nwk = _x.getSize() + 6*(_x.getSize()*_halfOrder+1);
	_wk.setSize(nwk);
	gcvspl(_x.get(),_y.get(),_weights.get(),_halfOrder,_x.getSize(),
		_coefficients.get(),_errorVariance,_wk.get(),ierr);
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 * All data members of the specified spline are copied.
 *
 * @param aSpline GCVSpline object to be copied.
 */
GCVSpline::
GCVSpline(const GCVSpline &aSpline) :
	Function(aSpline),
	_halfOrder(_propHalfOrder.getValueInt()),
	_errorVariance(_propErrorVariance.getValueDbl()),
	_x(_propX.getValueDblArray()),
	_weights(_propWeights.getValueDblArray()),
	_coefficients(_propCoefficients.getValueDblArray()),
	_wk(_propWk.getValueDblArray()),
	_workEval(0.0),
	_y(0.0)
{
	setEqual(aSpline);
}
//_____________________________________________________________________________
/**
 * Copy this object.
 *
 * @return Pointer to a copy of this object.
 */
Object* GCVSpline::
copy() const
{
	GCVSpline *spline = new GCVSpline(*this);
	return(spline);
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set all member variables to their NULL or default values.
 */
void GCVSpline::
setNull()
{
	setType("GCVSpline");
	setupProperties();
	_knotIndex = 0;
}
//_____________________________________________________________________________
/**
 * Set up the serialized member variables.  This involves both generating
 * the properties and connecting them to the local pointers used to access
 * the serialized member variables.
 */
void GCVSpline::
setupProperties()
{
//const char GCVSpline::PROP_WK[] = "wk";

	// HALF ORDER
	_propHalfOrder.setName("half_order");
	_propHalfOrder.setValue(0);
	_propertySet.append( &_propHalfOrder );

	// ERROR VARIANCE
	_propErrorVariance.setName("error_variance");
	_propErrorVariance.setValue(0.0);
	_propertySet.append( &_propErrorVariance );

	// X- INDEPENDENT VARIABLES
	_propX.setName("x");
	Array<double> x(0.0);
	_propX.setValue(x);
	_propertySet.append( &_propX );

	// WEIGHTS
	_propWeights.setName("weights");
	Array<double> weights(1.0);
	_propWeights.setValue(weights);
	_propertySet.append( &_propWeights );

	// COEFFICIENTS
	_propCoefficients.setName("coefficients");
	Array<double> coefs(0.0);
	_propCoefficients.setValue(coefs);
	_propertySet.append( &_propCoefficients );

	// WORK ARRAY
	_propWk.setName("wk");
	Array<double> wk(0.0);
	_propWk.setValue(wk);
	_propertySet.append( &_propWk );
}
//_____________________________________________________________________________
/**
 * Set all member variables equal to the members of another object.
 * Note that this method is private.  It is only meant for copying the data
 * members defined in this class.  It does not, for example, make any changes
 * to data members of base classes.
 */
void GCVSpline::
setEqual(const GCVSpline &aSpline)
{
	setNull();

	// VALUES
	_halfOrder = aSpline._halfOrder;
	_errorVariance = aSpline._errorVariance;
	_knotIndex = aSpline._knotIndex;

	// CHECK ARRAY SIZES
	if(aSpline.getSize()<=0) return;
	if(_halfOrder<=0) return;

	// ALLOCATE ARRAYS
	_x = aSpline._x;
	_y = aSpline._y;
	_weights = aSpline._weights;
	_coefficients = aSpline._coefficients;
	_wk = aSpline._wk;
	_workEval = aSpline._workEval;
}
//_____________________________________________________________________________
/**
 * Initialize the spline with X and Y values.
 *
 * @param aN the number of X and Y values
 * @param aXValues the X values
 * @param aYValues the Y values
 */
void GCVSpline::
init(Function* aFunction)
{
	if (aFunction == NULL)
		return;

	int degree = 5;
	int order = degree + 1;
	GCVSpline* gcv = dynamic_cast<GCVSpline*>(aFunction);
	if (gcv != NULL) {
		setEqual(*gcv);
	} else if (aFunction->getNumberOfPoints() == 0) {
		// A GCVSpline must have at least getOrder() data points.
		// If aFunction is a Constant, use its Y value for all data points.
		// If it is not, make up the data points.
		double* x = new double[order];
		double* y = new double[order];
		for (int i=0; i<order; i++)
			x[i] = i;
		Constant* cons = dynamic_cast<Constant*>(aFunction);
		if (cons != NULL) {
			for (int i=0; i<order; i++)
				y[i] = cons->evaluate();
		} else {
			for (int i=0; i<order; i++)
				y[i] = 1.0;
		}
		*this = GCVSpline(degree, order, x, y);
		delete x;
		delete y;
	} else if (aFunction->getNumberOfPoints() < order) {
		// A GCVSpline must have at least getOrder() data points.
		// Use as many data points as aFunction has, and then fill
		// in the rest by copying the last Y value, and incrementing
		// the X value by the step between the last two real data points.
		double* x = new double[order];
		double* y = new double[order];
		double step = 1.0;
		if (aFunction->getNumberOfPoints() >= 2)
			step = aFunction->getXValues()[aFunction->getNumberOfPoints()-1] -
			   aFunction->getXValues()[aFunction->getNumberOfPoints()-2];
		for (int i=0; i<aFunction->getNumberOfPoints(); i++) {
			x[i] = aFunction->getXValues()[i];
			y[i] = aFunction->getYValues()[i];
		}
		for (int i=aFunction->getNumberOfPoints(); i<order; i++) {
			x[i] = x[i-1] + step;
			y[i] = y[i-1];
		}
		*this = GCVSpline(degree, order, x, y);
		delete x;
		delete y;
	} else {
		*this = GCVSpline(degree, aFunction->getNumberOfPoints(),
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
GCVSpline& GCVSpline::
operator=(const GCVSpline &aSpline)
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
// DEGREE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the degree of this spline.
 *
 * @param aDegree Degree of spline.  Legal values: 1 = linear, 3 = cubic,
 * 5 = quintic, 7 = heptic.
 */
void GCVSpline::
setDegree(int aDegree)
{
	_halfOrder = (aDegree + 1) / 2;

	// TOO SMALL
	if(_halfOrder<1) {
		printf("GCVSpline.setDegree: WARN- invalid half order %d.\n",
			_halfOrder);
		printf("\tSetting degree = 1 (linear spline.)\n");
		_halfOrder = 1;
	}

	// TOO LARGE
	if(_halfOrder>4) {
		printf("GCVSpline.setDegree: WARN- invalid half order %d.\n",
			_halfOrder);
		printf("\tSetting degree = 7 (heptic spline.)\n");
		_halfOrder = 4;
	}
}
//_____________________________________________________________________________
/**
 * Get the degree of this spline.
 *
 * @return Degree of spline: 1 = linear, 3 = cubic, 5 = quintic, 7 = heptic.
 */
int GCVSpline::
getDegree() const
{
	int degree = 2*_halfOrder - 1;
	return(degree);
}

//-----------------------------------------------------------------------------
// ORDER
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the order of this spline.
 *
 * @return Order of spline: 2 = linear, 4 = cubic, 6 = quintic, 8 = heptic.
 */
int GCVSpline::
getOrder() const
{
	int order = 2*_halfOrder;
	return(order);
}

//-----------------------------------------------------------------------------
// HALF ORDER
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the half order of this spline.
 *
 * @return Half order of spline: 1 = linear, 2 = cubic, 3 = quintic, 4 = heptic.
 */
int GCVSpline::
getHalfOrder() const
{
	return(_halfOrder);
}

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
int GCVSpline::
getSize() const
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
double GCVSpline::
getMinX() const
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
double GCVSpline::
getMaxX() const
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
const Array<double>& GCVSpline::
getX() const
{
	return(_x);
}
//_____________________________________________________________________________
/**
 * Get the array of independent variables used to construct the spline.
 * For the number of independent variable data points use getN().
 *
 * @return Pointer to the independent variable data points.
 * @see getN();
 */
const double* GCVSpline::
getXValues() const
{
	return(&_x[0]);
}
//_____________________________________________________________________________
/**
 * Get the array of dependent variables used to construct the spline.
 *
 * @return Pointer to the dependent variable data points.
 */
const double* GCVSpline::
getYValues() const
{
	return(&_y[0]);
}
//_____________________________________________________________________________
/**
 * Get the array of coefficients for the spline.
 * For the number of coefficients use getNX().
 *
 * @return Pointer to the coefficients.
 * @see getCoefficients();
 */
const Array<double>& GCVSpline::
getCoefficients() const
{
	return(_coefficients);
}



//=============================================================================
// EVALUATION
//=============================================================================
//_____________________________________________________________________________
/**
 * Update the bounding box for this function.
 *
 * For an GCVSpline, there is only one indepdendent variable x, so the 
 * minimum and maximum values of indepdent variables y and z is 0.0.
 *
 * When this method is called, the minimum and maximum x of the bounding box
 * is simply set to the minimum and maximum values of the x data points that
 * were used to construct the spline, that is, min x = x[0] and
 * max x = x[getN()-1].
 *
 * @see Function
 */
void GCVSpline::
updateBoundingBox()
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

double GCVSpline::
getX(int aIndex) const
{
	if (aIndex >= 0 && aIndex < _x.getSize())
		return _x.get(aIndex);
	else {
		throw Exception("GCVSpline::getX(): index out of bounds.");
		return 0.0;
	}
}

double GCVSpline::
getY(int aIndex) const
{
	if (aIndex >= 0 && aIndex < _y.getSize())
		return _y.get(aIndex);
	else {
		throw Exception("GCVSpline::getY(): index out of bounds.");
		return 0.0;
	}
}

void GCVSpline::
setX(int aIndex, double aValue)
{
	if (aIndex >= 0 && aIndex < _x.getSize()) {
		_x[aIndex] = aValue;
	   int ierr=0;
	   gcvspl(_x.get(),_y.get(),_weights.get(),_halfOrder,_x.getSize(),
		   _coefficients.get(),_errorVariance,_wk.get(),ierr);
	} else {
		throw Exception("GCVSpline::setX(): index out of bounds.");
	}
}

void GCVSpline::
setY(int aIndex, double aValue)
{
	if (aIndex >= 0 && aIndex < _y.getSize()) {
		_y[aIndex] = aValue;
	   int ierr=0;
	   gcvspl(_x.get(),_y.get(),_weights.get(),_halfOrder,_x.getSize(),
		   _coefficients.get(),_errorVariance,_wk.get(),ierr);
	} else {
		throw Exception("GCVSpline::setY(): index out of bounds.");
	}
}

void GCVSpline::
scaleY(double aScaleFactor)
{
	for (int i = 0; i < _y.getSize(); i++)
		_y[i] *= aScaleFactor;

	// Recalculate the coefficients
	int ierr=0;
	gcvspl(_x.get(),_y.get(),_weights.get(),_halfOrder,_x.getSize(),
		_coefficients.get(),_errorVariance,_wk.get(),ierr);
}

bool GCVSpline::
deletePoint(int aIndex)
{
	int minNumPoints = getOrder();

	if (_x.getSize() > minNumPoints && _y.getSize() > minNumPoints &&
		 _weights.getSize() > minNumPoints && _coefficients.getSize() > minNumPoints &&
		 aIndex < _x.getSize() && aIndex < _y.getSize() &&
		 aIndex < _weights.getSize() && aIndex < _coefficients.getSize()) {
      _x.remove(aIndex);
	   _y.remove(aIndex);
	   _weights.remove(aIndex);
	   _coefficients.remove(aIndex);
	   int nwk = _x.getSize() + 6*(_x.getSize()*_halfOrder+1);
	   _wk.setSize(nwk);

	   // Recalculate the coefficients
	   int ierr=0;
	   gcvspl(_x.get(),_y.get(),_weights.get(),_halfOrder,_x.getSize(),
		       _coefficients.get(),_errorVariance,_wk.get(),ierr);
		return true;
   }

   return false;
}

bool GCVSpline::
deletePoints(const Array<int>& indices)
{
	bool pointsDeleted = false;
	int minNumPoints = getOrder();
	int numPointsLeft = _x.getSize() - indices.getSize();

	if (numPointsLeft >= minNumPoints) {
		// Assume the indices are sorted highest to lowest
		for (int i=0; i<indices.getSize(); i++) {
			int index = indices.get(i);
			if (index >= 0 && index < _x.getSize()) {
	         _x.remove(index);
	         _y.remove(index);
				_weights.remove(index);
				_coefficients.remove(index);
				pointsDeleted = true;
			}
		}

		if (pointsDeleted) {
	      int nwk = _x.getSize() + 6*(_x.getSize()*_halfOrder+1);
	      _wk.setSize(nwk);

	      // Recalculate the coefficients
	      int ierr=0;
	      gcvspl(_x.get(),_y.get(),_weights.get(),_halfOrder,_x.getSize(),
		          _coefficients.get(),_errorVariance,_wk.get(),ierr);
		}
	}

   return pointsDeleted;
}

int GCVSpline::
addPoint(double aX, double aY)
{
	int i;
	for (i=0; i<_x.getSize(); i++)
		if (_x[i] > aX)
			break;

	_x.insert(i, aX);
	_y.insert(i, aY);
	if (i == _x.getSize()) {
	   _weights.insert(i, _weights[i-1]);
	   _coefficients.insert(i, _coefficients[i-1]);
	} else {
	   _weights.insert(i, _weights[i]);
	   _coefficients.insert(i, _coefficients[i]);
	}
	int nwk = _x.getSize() + 6*(_x.getSize()*_halfOrder+1);
	_wk.setSize(nwk);

	// Recalculate the coefficients
	int ierr=0;
	gcvspl(_x.get(),_y.get(),_weights.get(),_halfOrder,_x.getSize(),
		_coefficients.get(),_errorVariance,_wk.get(),ierr);

	return i;
}

//_____________________________________________________________________________
/**
 * Evaluate this function or a derivative of this function given a set of
 * independent variables.  Only splines of one dimension are supported by
 * this class, so values of independent variables y and z are ignored.
 *
 * @param aDerivOrder Derivative order.  If aDerivOrder == 0, the function
 * is evaluated.  Otherwise, if aDerivOrder > 0, the aDerivOrder'th
 * derivative of the function is evaluated.  For example, if aDerivOrder == 1,
 * the first derivative of the function is returned.  Negative values of
 * aDerivOrder (integrals of the function) are not supported.
 * @param aX Value of the x independent variable at which to evaluate
 * this function or its derivatives.
 * @param aY Value of the y independent variable at which to evaluate
 * this function or its derivatives (ignored).
 * @param aZ Value of the z independent variable at which to evaluate
 * this function or its derivatives (ignored).
 * @return Value of the function or one of its derivatives.
 */
double GCVSpline::
evaluate(int aDerivOrder,double aX,double aY,double aZ)
{
	// NOT A NUMBER
	// The following seemingly innocent line cost 70% of the compute time
	// for this method.  It was constructing and destructing an Array<double>
	//if(_coefficients==NULL) return(rdMath::NAN);
	if(aX<_x[0] || aX>_x.getLast()){
		char msg[256];
		sprintf(msg, "ERROR - Evaluating GCVSpline at %g outside its domain [%g, %g]", 
						aX, _x[0], _x.getLast());
		throw(Exception(msg, __FILE__,__LINE__));
	}
	if(aDerivOrder<0)
		throw(Exception("ERROR - Evaluating GCVSpline with negative derivative order", __FILE__,__LINE__));

	// EVALUATE
	double value;
	if(aDerivOrder>getDegree()) {
		value = 0.0;
	} else {
		value = splder(aDerivOrder,_halfOrder,getSize(),aX,
			&_x[0],&_coefficients[0],&_knotIndex,&_workEval[0]);
	}

	return(value);
}

Array<XYPoint>* GCVSpline::renderAsLineSegments(int aIndex)
{
	if (aIndex < 0 || aIndex >= getNumberOfPoints() - 1)
		return NULL;

	Array<XYPoint>* xyPts = new Array<XYPoint>(XYPoint());

   // X sometimes goes slightly beyond the range due to roundoff error,
	// so do the last point separately.
	int numSegs = 20;
	for (int i=0; i<numSegs-1; i++) {
		double x = _x[aIndex] + (double)i * (_x[aIndex + 1] - _x[aIndex]) / ((double)numSegs - 1.0);
		xyPts->append(XYPoint(x, evaluate(0, x, 0.0, 0.0)));
	}
	xyPts->append(XYPoint(_x[aIndex + 1], evaluate(0, _x[aIndex + 1], 0.0, 0.0)));

	return xyPts;
}
