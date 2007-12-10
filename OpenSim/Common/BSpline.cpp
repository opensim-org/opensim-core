// BSpline.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2007, Stanford University. All rights reserved. 
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
 * Author:  Jeff Reinbolt
 
 */



// C++ INCLUDES
#include "BSpline.h"

using namespace OpenSim;
//using namespace std;


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
BSpline::~BSpline()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
BSpline::BSpline()
{
	_order			= 5;								// order of b-spline
	_nControlPoints	= 10;								// number of control points
	_nCurvePoints	= 20;								// number of curve points
	_nDimensions	= 2;								// number of dimensions for control points and curve points
	this->createComponents(_order, _nControlPoints, _nCurvePoints); // create b-spline components
}
//_____________________________________________________________________________
/**
 * TODO: add comment
 */
BSpline::BSpline(int o, int ncp1, int ncp2, int nd)
{
	_order			= o;								// order of b-spline
	_nControlPoints	= ncp1;								// number of control points
	_nCurvePoints	= ncp2;								// number of curve points
	_nDimensions	= nd;								// number of dimensions for control points and curve points
	this->createComponents(_order, _nControlPoints, _nCurvePoints); // create b-spline components
}
//_____________________________________________________________________________
//=============================================================================
// CONSTRUCTION
//=============================================================================
/**
 * TODO: add comment
 */
void BSpline::createComponents(int o, int ncp1, int ncp2)
{
	_order = o;
	_nControlPoints = ncp1;
	_nCurvePoints = ncp2;
	if(_order!=(_nParametricKnots-_nControlPoints-1)		// check if order has changed ...
		&&(_nControlPoints*_nDimensions)==(int)_controlPoints.size() // ... number of control points is same ...
		&&_nParametricKnots==(int)_parametricKnots.size()) // ... number of parametric knots is same
	{
		if(_nControlPoints<_order){ _nControlPoints = _order+1; } // check if sufficient control points for order of the curve
		_nParametricKnots = _order+_nControlPoints+1;		// number of parametric knots
	}
	if((_nControlPoints*_nDimensions)!=(int)_controlPoints.size()) // check if number of control points has changed
	{ 
		if(_nControlPoints<_order){ _nControlPoints = _order+1; } // check if sufficient control points for order of the curve
		_nParametricKnots = _order+_nControlPoints+1;		// number of parametric knots
		_controlPoints.resizeKeep(_nControlPoints, _nDimensions); // control points (matrix)
	}
	if(_nParametricKnots!=(int)_parametricKnots.size())	// check if number of parametric knots has changed
	{ 
		_parametricKnots.resizeKeep(_nParametricKnots);	// parametric knots (vector)
		this->createParametricKnots();					// create parametric knots
	}
	if((_nCurvePoints*_nDimensions)!=(int)_curvePoints.size()) // check if number of curve points has changed
	{ 
		_curvePoints.resizeKeep(_nCurvePoints, _nDimensions); // curve points (matrix)
		_dataPoints.resizeKeep(_nCurvePoints, _nDimensions); // data points (matrix)
	}
	if((_nCurvePoints*_nControlPoints)!=(int)_blendingFunction.size()) // check if number of curve points and/or control points has changed
	{ 
		_blendingFunction.resizeKeep(_nCurvePoints, _nControlPoints); // blending function (matrix)
		this->createBlendingFunction();					// create blending function 
	}
}
//_____________________________________________________________________________
/**
 * Create parametric knots (or knot values) for open curve B-spline,
 * which relate the parametric variable u to the control points.
 */
void BSpline::createParametricKnots()
{	
	int knot = 0;
	for(int k=0; k<_nParametricKnots; k++)
	{				
		if(k>_order)
		{
			if(k<=_nControlPoints){ _parametricKnots(k) = ++knot; }
			else{ _parametricKnots(k) = knot; }
		}
		else{ _parametricKnots(k) = knot; }		
	}
}
//_____________________________________________________________________________
/**
 * Create blending function for open curve B-spline,
 * which relate each curve point to the control points.
 */
void BSpline::createBlendingFunction()
{	
	double tol = 1e-8;									// equality tolerance
	double u = 0.0;										// parametric variable u
	int uMax = _nControlPoints - _order;				// maximum value of parametric variable u
	int degree = _order+1;								// degree of normalized polynomial (degree <= number of control points)

	for(int p=0; p<_nCurvePoints; p++)					// loop through each curve point
	{
		double percentage = (double) p/(_nCurvePoints-1);
		u =	uMax*percentage;							// define parametric variable u for current curve point
		for(int d=0; d<degree; d++)						// loop through each normalized polynomial degree in u
		{
			if(d==0)									// check if normalized polynomial degree in u is 1 (indexed as 0)
			{											// calculate first order basis functions
				for(int n=0; n<_nControlPoints; n++)	// loop through each control point
				{
					if(u>=_parametricKnots(n) && u<_parametricKnots(n+1))
					{ 
						_blendingFunction(p, n) = 1.0; 
					}
					else if(abs(u-uMax)<tol && abs(u-_parametricKnots(n)-(uMax/_parametricKnots(_nParametricKnots-1)))<tol && abs(u-_parametricKnots(n+1))<tol)
					{ 
						_blendingFunction(p, n) = 1.0; 
					}
					else
					{ 
						_blendingFunction(p, n) = 0.0; 
					}
				}										// end loop through each control point
			}
			else										// otherwise if the normalized polynomial degree in u is NOT 1 (indexed as 0)
			{											// calculate the higher order basis functions
				for(int n=0; n<_nControlPoints; n++)	// loop through each control point
				{
					if((_parametricKnots(n+d)-_parametricKnots(n))==0 && (_parametricKnots(n+d+1)-_parametricKnots(n+1))==0)
					{ 
						_blendingFunction(p, n) = 0.0; 
					}
					else if((_parametricKnots(n+d)-_parametricKnots(n))==0 && (_parametricKnots(n+d+1)-_parametricKnots(n+1))!=0)
					{ 
						_blendingFunction(p, n) = ((_parametricKnots(n+d+1)-u)*_blendingFunction(p, n+1))/(_parametricKnots(n+d+1)-_parametricKnots(n+1));
					}
					else if((_parametricKnots(n+d)-_parametricKnots(n))!=0 && (_parametricKnots(n+d+1)-_parametricKnots(n+1))==0)
					{
						_blendingFunction(p, n) = ((u-_parametricKnots(n))*_blendingFunction(p, n))/(_parametricKnots(n+d)-_parametricKnots(n));
					}
					else
					{
						_blendingFunction(p, n) = (((u-_parametricKnots(n))*_blendingFunction(p, n))/(_parametricKnots(n+d)-_parametricKnots(n))) + (((_parametricKnots(n+d+1)-u)*_blendingFunction(p, n+1))/(_parametricKnots(n+d+1)-_parametricKnots(n+1)));
					}
				}										// end loop through each control point
			}
		}												// end of loop through each polynomial degree in u
	}													// end of loop through range of the parametric variable u
}
//_____________________________________________________________________________
/**
 * TODO: add comment
 */
void BSpline::createCurvePoints()
{
	_curvePoints = _blendingFunction*_controlPoints;	// compute curve points from blending function and control points
}
//_____________________________________________________________________________
/**
 * TODO: add comment
 */
void BSpline::createControlPoints()
{
	_controlPoints = ((~_blendingFunction*_blendingFunction).invert())*(~_blendingFunction*_dataPoints);	// compute control points from linear regression using blending function and data points (to be best-fit)
}
//_____________________________________________________________________________
//=============================================================================
// OPERATORS
//=============================================================================
//=============================================================================
// SET AND GET
//=============================================================================
//-----------------------------------------------------------------------------
// ORDER
//-----------------------------------------------------------------------------
/**
 * TODO: add comment
 */
void BSpline::setOrder(int o)
{ 
	_order	= o;										// set the order of the b-spline
	this->createComponents(_order, _nControlPoints, _nCurvePoints); // create b-spline components
}					
//_____________________________________________________________________________
/**
 * TODO: add comment
 */
int BSpline::getOrder()
{ 
	return _order;										// returns the order of the b-spline
}					
//_____________________________________________________________________________
//-----------------------------------------------------------------------------
// CONTROL POINTS
//-----------------------------------------------------------------------------
/**
 * TODO: add comment
 */
void BSpline::setNControlPoints(int ncp)  
{ 
	_nControlPoints = ncp;								// set the number of control points
	this->createComponents(_order, _nControlPoints, _nCurvePoints); // create b-spline components
}				
//_____________________________________________________________________________
/**
 * TODO: add comment
 */
int BSpline::getNControlPoints()
{ 
	return _nControlPoints;								// returns the number of control points
}					
//_____________________________________________________________________________
/**
 * TODO: add comment
 */
void BSpline::setControlPoint(int r, int c, SimTK::Real value)
{
	_controlPoints(r, c) = value;						// set the indexed control point value
}
//_____________________________________________________________________________
/**
 * TODO: add comment
 */
SimTK::Real BSpline::getControlPoint(int r, int c)
{ 
	return _controlPoints(r, c);						// returns the specified control point value
}						
//_____________________________________________________________________________
/**
 * TODO: add comment
 */
void BSpline::setControlPoints(SimTK::Matrix cp)
{
	_controlPoints = cp;								// set all control point values
}
//_____________________________________________________________________________
/**
 * TODO: add comment
 */
SimTK::Matrix BSpline::getControlPoints()
{ 
	return _controlPoints;								// returns the control points (matrix)
}
//_____________________________________________________________________________
//-----------------------------------------------------------------------------
// PARAMETRIC KNOT VECTOR
//-----------------------------------------------------------------------------
/**
 * TODO: add comment
 */
	// Setting the number of parametric knots is not possible
//_____________________________________________________________________________
/**
 * TODO: add comment
 */
int BSpline::getNParametricKnots()
{ 
	return _nParametricKnots;							// returns the number of parametric knots
}					
//_____________________________________________________________________________
/**
 * TODO: add comment
 */
	// Setting the knot vector value at index i is not possible
//_____________________________________________________________________________
/**
 * TODO: add comment
 */
SimTK::Real BSpline::getParametricKnotValue(int i)
{ 
	return _parametricKnots(i);							// returns the knot vector value at index i
}		
//_____________________________________________________________________________
/**
 * TODO: add comment
 */
	// Setting the knot vector is not possible
//_____________________________________________________________________________
/**
 * TODO: add comment
 */
SimTK::Vector BSpline::getParametricKnots()
{ 
	return _parametricKnots;							// returns the parametric knots (vector)
}
//_____________________________________________________________________________
//-----------------------------------------------------------------------------
// CURVE POINTS
//-----------------------------------------------------------------------------
/**
 * TODO: add comment
 */
void BSpline::setNCurvePoints(int ncp)  
{ 
	_nCurvePoints = ncp;								// set the number of curve points
	this->createComponents(_order, _nControlPoints, _nCurvePoints); // create b-spline components
}
//_____________________________________________________________________________
/**
 * TODO: add comment
 */
int BSpline::getNCurvePoints()
{ 
	return _nCurvePoints;								// returns the number of curve points
}				
//_____________________________________________________________________________
/**
 * TODO: add comment
 */
	// Setting the specified curve point value is not possible
//_____________________________________________________________________________
/**
 * TODO: add comment
 */
SimTK::Real BSpline::getCurvePoint(int r, int c)
{ 
	this->createCurvePoints();							// compute curve points
	return _curvePoints(r, c);							// returns the specified curve point value
}						
//_____________________________________________________________________________
/**
 * TODO: add comment
 */
	// Setting the curve points is not possible
//_____________________________________________________________________________
/**
 * TODO: add comment
 */
SimTK::Matrix BSpline::getCurvePoints()
{ 
	this->createCurvePoints();							// compute curve points
	return _curvePoints;								// returns the curve points (matrix)
}
//_____________________________________________________________________________
//-----------------------------------------------------------------------------
// DATA POINTS
//-----------------------------------------------------------------------------
/**
 * TODO: add comment
 */
void BSpline::setDataPoint(int r, int c, SimTK::Real value)
{
	_dataPoints(r, c) = value;							// set the indexed control point value
	this->createControlPoints();						// compute control points (to be best-fit)
}
//_____________________________________________________________________________
/**
 * TODO: add comment
 */
SimTK::Real BSpline::getDataPoint(int r, int c)
{ 
	return _dataPoints(r, c);							// returns the specified data point value
}						
//_____________________________________________________________________________
/**
 * TODO: add comment
 */
void BSpline::setDataPoints(SimTK::Matrix dp)
{
	_dataPoints = dp;									// set all data point values
	this->createControlPoints();						// compute control points (to be best-fit)
}
//_____________________________________________________________________________
/**
 * TODO: add comment
 */
SimTK::Matrix BSpline::getDataPoints()
{ 
	return _dataPoints;									// returns the data points (matrix)
}
//_____________________________________________________________________________
//=============================================================================
// EVALUATION
//=============================================================================
