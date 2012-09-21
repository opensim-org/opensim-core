#ifndef _GCVSpline_h_
#define _GCVSpline_h_
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  GCVSpline.h                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

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
 * any class requiring an Fuction as input.
 *
 * @author Frank C. Anderson
 */
namespace OpenSim { 

class OSIMCOMMON_API GCVSpline : public Function {
OpenSim_DECLARE_CONCRETE_OBJECT(GCVSpline, Function);

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
	/** Spline Y values. */
	PropertyDblArray _propY;

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

	/** Y (dependent) values of the function. These are called aF in the
	constructor and are stored here so that the function can be scaled
	later on. */
	Array<double> &_y;
	/** A workspace used when calculating derivatives of the spline. */
	mutable std::vector<int> _workDeriv;

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

	virtual void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber=-1);
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
	virtual double getMinX() const;
	virtual double getMaxX() const;
	virtual bool deletePoint(int aIndex);
	virtual bool deletePoints(const Array<int>& indices);
	virtual int addPoint(double aX, double aY);
	SimTK::Function* createSimTKFunction() const;

	//--------------------------------------------------------------------------
	// EVALUATION
	//--------------------------------------------------------------------------

//=============================================================================
};	// END class GCVSpline

}; //namespace
//=============================================================================
//=============================================================================

#endif  // __GCVSpline_h__
