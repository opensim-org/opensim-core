#ifndef _GCVSpline_h_
#define _GCVSpline_h_
// GCVSpline.h
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



// INCLUDES
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
 * A class for representing a smooth function with a generalized
 * cross-validation spline.  Linear, cubic, qunitic, and heptic splines
 * are supported:
 *
 *    m (half-order)     order         degree         description
 *    1                    2             1              linear
 *    2                    4             3              cubic
 *    3                    6             5              quintic
 *    4                    8             7              heptic
 *
 * This class wraps the gcvspl.c source code written by D. Twisk in 1994,
 * which is based on the GCVSPL code written in Fortran by Woltring
 * in 1985_07_04.  This class was initially based on a spline class
 * authored by Darryl Thelen and Victor Ng; it has been rewritten to fit
 * into the Realistic Dynamics, Inc. software framework.
 *
 * See the following source for details on how the GCV spline is fit:
 * Woltring, H.J. (1986).  A Fortran package for generalized,
 * cross-validatory spline smoothing and differentiation.  Advances in
 * Engineering Software, Vol. 8, No. 2, 104-113.
 *
 * This class inherits from Function and so can be used as input to
 * any class requiring an rdFuction as input.
 *
 * @author Frank C. Anderson
 */
namespace OpenSim { 

class OSIMCOMMON_API GCVSpline : public Function
{
//=============================================================================
// MEMBER VARIABLES
//=============================================================================
protected:
	// PROPERTIES
	/** Half order of the spline (degree+1)/2. */
	PropertyInt _propHalfOrder;
	/** Error variance for the data and spline fit.  The smoothing factor
	p is computed based on the error variance. */
	PropertyDbl _propErrorVariance;
	/** Array of values for the independent variables (i.e., the spline knot
	sequence).  This array must be monotonically increasing. */
	PropertyDblArray _propX;
	/** Array of weight values, one for each data point. */
	PropertyDblArray _propWeights;
	/** Spline coefficients. */
	PropertyDblArray _propCoefficients;
	/** Work array for construction of the spline. */
	PropertyDblArray _propWk;

	// REFERENCES
	/** Reference to the value of the HalfOrder property. */
	int &_halfOrder;
	/** Reference to the value of the ErrorVariance property. */
	double &_errorVariance;
	/** Reference to the value of the X property. */
	Array<double> &_x;
	/** Reference to the value of the Weights property. */
	Array<double> &_weights;
	/** Reference to the value of the Coefficients property. */
	Array<double> &_coefficients;
	/** Reference to the value of the Wk property. */
	Array<double> &_wk;


	/** Work array for evaluating the spline. */
	Array<double> _workEval;
	/** Knot index used to fascilitate finding the appropriate knot during
	an evaluation. */
	int _knotIndex;

	/** Y (dependent) values of the function. These are called aF in the
	constructor and are stored here so that the function can be scaled
	later on. */
	Array<double> _y;

//=============================================================================
// METHODS
//=============================================================================
public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	GCVSpline();
	GCVSpline(int aDegree,int aN,const double *aTimes,const double *aValues,
		const std::string &aName="",double aErrorVariance=0.0);
	GCVSpline(const GCVSpline &aSpline);
	virtual ~GCVSpline();
	virtual Object* copy() const;
private:
	void setNull();
	void setupProperties();
	void setEqual(const GCVSpline &aSpline);
	virtual void init(Function* aFunction);

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
	GCVSpline& operator=(const GCVSpline &aSpline);

	//--------------------------------------------------------------------------
	// SET AND GET
	//--------------------------------------------------------------------------
	void setDegree(int aDegree);
public:
	int getDegree() const;
	int getOrder() const;
	int getHalfOrder() const;
	int getSize() const;
	double getMinX() const;
	double getMaxX() const;
	const Array<double>& getX() const;
	virtual const double* getXValues() const;
	virtual const double* getYValues() const;
	const Array<double>& getCoefficients() const;
	virtual int getNumberOfPoints() const { return _x.getSize(); }
	virtual double getX(int aIndex) const;
	virtual double getY(int aIndex) const;
	virtual double getZ(int aIndex) const { return 0.0; }
	virtual void setX(int aIndex, double aValue);
	virtual void setY(int aIndex, double aValue);
	virtual void scaleY(double aScaleFactor);
	virtual bool deletePoint(int aIndex);
	virtual bool deletePoints(const Array<int>& indices);
	virtual void addPoint(double aX, double aY);
	virtual Array<XYPoint>* renderAsLineSegments(int aIndex);

	//--------------------------------------------------------------------------
	// EVALUATION
	//--------------------------------------------------------------------------
	virtual void updateBoundingBox();
	virtual double
		evaluate(int aDerivOrder,double aX=0.0,double aY=0.0,double aZ=0.0);

	OPENSIM_DECLARE_DERIVED(GCVSpline, Function);

//=============================================================================
};	// END class GCVSpline

}; //namespace
//=============================================================================
//=============================================================================

#endif  // __GCVSpline_h__
