// VectorGCVSplineR1R3.cpp
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
	if (_splineY0) delete _splineY0; _splineY0=NULL;
	if (_splineY1) delete _splineY1; _splineY1=NULL;
	if (_splineY2) delete _splineY2; _splineY2=NULL;
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
	if (aVectorSpline.getSplineY0()==NULL) _splineY0=NULL;	else _splineY0=new GCVSpline(*aVectorSpline.getSplineY0());
	if (aVectorSpline.getSplineY1()==NULL) _splineY1=NULL;	else _splineY1=new GCVSpline(*aVectorSpline.getSplineY1());
	if (aVectorSpline.getSplineY2()==NULL) _splineY2=NULL;	else _splineY2=new GCVSpline(*aVectorSpline.getSplineY2());

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
calcValue(const double *aX,double *rY, int aSize)
{
	rY[0] = _splineY0->calcValue(SimTK::Vector(1, aX));
	rY[1] = _splineY1->calcValue(SimTK::Vector(1, aX));
	rY[2] = _splineY2->calcValue(SimTK::Vector(1, aX));
}
//_____________________________________________________________________________
/**
 * Evaluate this function given a value for the independent variable.  
 *
 * @param aX Vector of the independent variables.
 * @param rY Vector of the resulting dependent variables.
 */
void VectorGCVSplineR1R3::
calcValue(const Array<double> &aX,Array<double> &rY)
{
	assert(aX.getSize()==1);
	assert(rY.getSize()==3);
	calcValue(&aX[0],&rY[0], 1);
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
calcDerivative(const Array<double> &aX,Array<double> &rY,
			const Array<int> &aDerivWRT)
{
	assert(aX.getSize()==1);

	int derivOrder = aDerivWRT.getSize();
	std::vector<int> derivComponents;
	for (int i=0; i< derivOrder; i++) 
		derivComponents.push_back(aDerivWRT.get(i));
	SimTK::Vector arg(1, aX[0]);
	rY[0] = _splineY0->calcDerivative(derivComponents,arg);
	rY[1] = _splineY1->calcDerivative(derivComponents,arg);
	rY[2] = _splineY2->calcDerivative(derivComponents,arg);
}


