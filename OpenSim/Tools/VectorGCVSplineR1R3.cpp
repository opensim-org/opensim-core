// VectorGCVSplineR1R3.cpp
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

/*  
 * Author:  Frank C. Anderson and Saryn R. Goldberg
 
 */


// C++ INCLUDES
#include "VectorGCVSplineR1R3.h"


//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________


using namespace OpenSim;
/**
 * Destructor.
 */
VectorGCVSplineR1R3::~VectorGCVSplineR1R3()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
VectorGCVSplineR1R3::
VectorGCVSplineR1R3() : _value(0.0)
{
	setNull();
}
//_____________________________________________________________________________
/**
 * Construct a spline of a specified degree given arrays of paired data points
 * (x,f1(x),f2(x),f3(x)).
 *
 * @param aDegree Degree of the spline.  Only the following degrees
 * are supported: 1 = linear, 3 = cubic, 5 = qunitic, and 7 = heptic.
 * @param aN  of data points.
 * @param aX Array of independent values- should be aN long.
 * @param aF Array of function values- should be aN long and have 3 columns.
 * @param aName Optional name of the spline.
 * @param aErrorVariance Estimate of the variance of the error in the data to
 * be fit.  If negative, the variance will be estimated.  If 0.0, the fit will
 * try to fit the data points exactly- no smoothing.  If positive, the fit
 * will be smoothed according to the specified variance. The larger the error
 * variance, the more the smoothing.  The smoothing parameter, p, in
 * Woltring (1986) is computed based on the error variance.
 */
VectorGCVSplineR1R3::
VectorGCVSplineR1R3(int aDegree,int aN,const double *aX,double *aY0,double *aY1,
	double *aY2,const char *aName,double aErrorVariance) : _value(0.0)
{
	setNull();

	_splineY0 = new GCVSpline(aDegree, aN, aX, aY0, "spline_0", aErrorVariance);
	_splineY1 = new GCVSpline(aDegree, aN, aX, aY1, "spline_1", aErrorVariance);
	_splineY2 = new GCVSpline(aDegree, aN, aX, aY2, "spline_2", aErrorVariance);

}
//_____________________________________________________________________________
/**
 * Copy constructor.
 * All data members of the specified spline are copied.
 *
 * @param aSpline VectorGCVSplineR1R3 object to be copied.
 */
VectorGCVSplineR1R3::
VectorGCVSplineR1R3(const VectorGCVSplineR1R3 &aVectorSpline) :
	VectorFunction(aVectorSpline), _value(0.0)
{
	setEqual(aVectorSpline);
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set all member variables to their NULL or default values.
 */
void VectorGCVSplineR1R3::
setNull()
{
	setType("VectorGCVSplineR1R3");
	_splineY0 = NULL;
	_splineY1 = NULL;
	_splineY2 = NULL;
	_value.setSize(3);
}

//_____________________________________________________________________________
/**
 * Set all member variables equal to the members of another object.
 * Note that this method is private.  It is only meant for copying the data
 * members defined in this class.  It does not, for example, make any changes
 * to data members of base classes.
 */
void VectorGCVSplineR1R3::
setEqual(const VectorGCVSplineR1R3 &aVectorSpline)
{
	setNull();

	// VALUES
	_splineY0 = aVectorSpline.getSplineY0();
	_splineY1 = aVectorSpline.getSplineY1();
	_splineY2 = aVectorSpline.getSplineY2();

}

//_____________________________________________________________________________
/**
 * Copy this object.
 *
 * @return Pointer to a copy of this object.
 */
Object* VectorGCVSplineR1R3::
copy() const
{
	VectorGCVSplineR1R3 *func = new VectorGCVSplineR1R3(*this);
	return(func);
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
VectorGCVSplineR1R3& VectorGCVSplineR1R3::
operator=(const VectorGCVSplineR1R3 &aVectorSpline)
{
	// BASE CLASS
	VectorFunction::operator=(aVectorSpline);

	// DATA
	setEqual(aVectorSpline);

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
 * 5 = qunitic, 7 = heptic.
 */
void VectorGCVSplineR1R3::
setDegree(int aDegree)
{
	_splineY0->setDegree(aDegree);
	_splineY1->setDegree(aDegree);
	_splineY2->setDegree(aDegree);

}
//-----------------------------------------------------------------------------
// SPLINES
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the individual splines in the vector.
 *
 */
GCVSpline* VectorGCVSplineR1R3::
getSplineY0() const
{
	return(_splineY0);
}
//_____________________________________________________________________________
/**
 * Get the individual splines in the vector.
 *
 */
GCVSpline* VectorGCVSplineR1R3::
getSplineY1() const
{
	return(_splineY1);
}
//_____________________________________________________________________________
/**
 * Get the individual splines in the vector.
 *
 */
GCVSpline* VectorGCVSplineR1R3::
getSplineY2() const
{
	return(_splineY2);
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
void VectorGCVSplineR1R3::
updateBoundingBox()
{
	setMinX(0.0);
	setMaxX(0.0);

	if(_splineY0->getSize()<=0) return;

	setMinX(_splineY0->getX().get(0));
	setMaxX(_splineY0->getX().getLast());
}
//_____________________________________________________________________________
/**
 * Evaluate this function given a value for the independent variable.  
 *
 * @param aX Vector of the independent variables.
 * @param rY Vector of the resulting dependent variables.
 */
void VectorGCVSplineR1R3::
evaluate(const double *aX,double *rY)
{
	rY[0] = _splineY0->evaluate(0,aX[0]);
	rY[1] = _splineY1->evaluate(0,aX[0]);
	rY[2] = _splineY2->evaluate(0,aX[0]);
}
//_____________________________________________________________________________
/**
 * Evaluate this function given a value for the independent variable.  
 *
 * @param aX Vector of the independent variables.
 * @param rY Vector of the resulting dependent variables.
 */
void VectorGCVSplineR1R3::
evaluate(const Array<double> &aX,Array<double> &rY)
{
	assert(aX.getSize()==1);
	evaluate(&aX[0],&rY[0]);
}
//_____________________________________________________________________________
/**
 * Evaluate this function or a derivative of this function given a value for the
 * independent variable.  
 *
 * @param aX Vector of the independent variables.
 * @param rY Vector of the resulting dependent variables.
 * @param aDerivWRT
 */
void VectorGCVSplineR1R3::
evaluate(const Array<double> &aX,Array<double> &rY,
			const Array<int> &aDerivWRT)
{
	assert(aX.getSize()==1);

	int derivOrder = aDerivWRT.getSize();
	rY[0] = _splineY0->evaluate(derivOrder,aX[0]);
	rY[1] = _splineY1->evaluate(derivOrder,aX[0]);
	rY[2] = _splineY2->evaluate(derivOrder,aX[0]);
}


