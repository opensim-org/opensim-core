#ifndef _BSpline_h_
#define _BSpline_h_
// BSpline.h
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

// INCLUDES
#include "SimTKcommon.h"
#include "osimCommonDLL.h"
#include <string>
#include "Array.h"
#include "PropertyInt.h"
#include "PropertyDbl.h"
#include "PropertyDblArray.h"
#include "Function.h"



//template class OSIMCOMMON_API Array<double>;


//=============================================================================
//=============================================================================
/**
 * A class for representing and evaluating univariate B-splines. 
 *
 * @author Jeff Reinbolt
 */


/******************************************************************************************************/
namespace OpenSim { 

class OSIMCOMMON_API BSpline : public Function
{
//=============================================================================
// MEMBER VARIABLES
//=============================================================================
protected:
	int	_order;											// order of b-spline
	int	_nControlPoints;								// number of control points
	int	_nParametricKnots;								// number of parametric knots
	int	_nCurvePoints;									// number of curve points
	int	_nDimensions;									// number of dimensions for control points and curve points
	SimTK::Matrix _controlPoints;						// control points (matrix)
	SimTK::Vector _parametricKnots;						// parametric knots (vector)
	SimTK::Matrix _blendingFunction;					// blending function (matrix)
	SimTK::Matrix _curvePoints;							// curve points (matrix)
	SimTK::Matrix _dataPoints;							// data points (matrix)

//=============================================================================
// METHODS
//=============================================================================
public :
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	BSpline();											// constructor
	BSpline(int, int, int, int);						// assignment constructor
	virtual ~BSpline();									// destructor
	//virtual Object* copy() const;						// returns a copy of this object

	void createComponents(int, int, int);				// create the b-spline components
	void createParametricKnots();						// create the parametric knot vector
	void createBlendingFunction();						// create the blending function
	void createCurvePoints();							// create the curve points
	void createControlPoints();							// create the control points

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
/*public:
	BSpline& operator=(const BSpline &aBSpline);*/	

	//--------------------------------------------------------------------------
	// SET AND GET
	//--------------------------------------------------------------------------
public:
	void setOrder(int);									// set the order of the b-spline
	int  getOrder();									// returns the order of the b-spline

	void setNControlPoints(int);						// set the number of control points
	int	 getNControlPoints();							// returns the number of control points

	void setControlPoint(int, int, SimTK::Real);		// set the value of a specified control point
	SimTK::Real getControlPoint(int, int);				// returns the specified control point value

	void setControlPoints(SimTK::Matrix);				// set the value of all control points
	SimTK::Matrix getControlPoints();					// returns the control points

	// Setting the number of parametric knots is not possible
	int	 getNParametricKnots();							// returns the number of parametric knots

	// Setting the knot vector value at index i is not possible
	SimTK::Real getParametricKnotValue(int);			// returns the knot vector value at index i

	// Setting the knot vector is not possible
	SimTK::Vector getParametricKnots();					// returns the knot vector

	void setNCurvePoints(int);							// set the number of curve points
	int  getNCurvePoints();								// returns the number of curve points

	// Setting the specified curve point value is not possible
	SimTK::Real getCurvePoint(int, int);				// returns the specified curve point value
	
	// Setting the curve points is not possible
	SimTK::Matrix getCurvePoints();						// returns the curve points

	void setDataPoint(int, int, SimTK::Real);			// set the value of a specified data point
	SimTK::Real getDataPoint(int, int);					// returns the specified data point value
	
	void setDataPoints(SimTK::Matrix);					// set the value of all data points
	SimTK::Matrix getDataPoints();						// returns the data points

	//--------------------------------------------------------------------------
	// EVALUATION
	//--------------------------------------------------------------------------

	OPENSIM_DECLARE_DERIVED(BSpline, Function);

//=============================================================================
};	// END class BSpline

}; //namespace
//=============================================================================
//=============================================================================

#endif  // __BSpline_h__
