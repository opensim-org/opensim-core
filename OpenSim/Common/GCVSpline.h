#ifndef OPENSIM_GCV_SPLINE_H_
#define OPENSIM_GCV_SPLINE_H_
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  GCVSpline.h                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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


// INCLUDES
#include "osimCommonDLL.h"
#include <string>
#include "PropertyInt.h"
#include "PropertyDbl.h"
#include "PropertyDblArray.h"
#include "Function.h"

// This class was initially based on a spline class
// authored by Darryl Thelen and Victor Ng; it has been rewritten to fit
// into the Realistic Dynamics, Inc. software framework.


//=============================================================================
//=============================================================================
namespace OpenSim { 

template <class T> class Array;

/**
 * A class for representing a smooth function with a generalized
 * cross-validation spline.  Linear, cubic, quintic, and heptic splines
 * are supported:
 *
 * @code
 *    m (half-order)     order         degree         description
 *    1                    2             1              linear
 *    2                    4             3              cubic
 *    3                    6             5              quintic
 *    4                    8             7              heptic
 * @endcode
 *
 * This class wraps the gcvspl.c source code written by D. Twisk in 1994,
 * which is based on the GCVSPL code written in Fortran by Woltring
 * in 1985_07_04.
 *
 * See the following source for details on how the GCV spline is fit:
 * Woltring, H.J. (1986).  A Fortran package for generalized,
 * cross-validatory spline smoothing and differentiation.  Advances in
 * Engineering Software, Vol. 8, No. 2, 104-113.
 *
 * @author Frank C. Anderson
 */
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
    /**
     * Construct a spline of a specified degree given arrays of paired data
     * points (x,f(x)). A name for the spline may be specified.
     *
     * @param aDegree Degree of the spline.  Only the following degrees
     * are supported: 1 = linear, 3 = cubic, 5 = quintic, and 7 = heptic.
     * @param aN Number of data points.
     * @param aX %Array of independent values- should be aN long.
     * @param aF %Array of function values- should be aN long.
     * @param aName Optional name of the spline.
     * @param aErrorVariance Estimate of the variance of the error in the data
     * to be fit.  If negative, the variance will be estimated.  If 0.0, the
     * fit will try to fit the data points exactly- no smoothing.  If positive,
     * the fit will be smoothed according to the specified variance. The larger
     * the error variance, the more the smoothing.  The smoothing parameter, p,
     * in Woltring (1986) is computed based on the error variance.
     */
    GCVSpline(int aDegree,int aN,const double *aX,const double *aF,
        const std::string &aName="",double aErrorVariance=0.0);
    GCVSpline(const GCVSpline &aSpline);
    virtual ~GCVSpline();

    void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber=-1) override;
private:
    void setNull();
    void setupProperties();
    void setEqual(const GCVSpline &aSpline);
    void init(Function* aFunction) override;

    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
public:
    GCVSpline& operator=(const GCVSpline &aSpline);

    //--------------------------------------------------------------------------
    // SET AND GET
    //--------------------------------------------------------------------------
    /**
     * %Set the degree of this spline.
     *
     * @param aDegree Degree of spline.  Legal values: 1 = linear, 3 = cubic,
     * 5 = quintic, 7 = heptic.
     */
    void setDegree(int aDegree);
public:
    int getDegree() const;
    /**
     * Get the order of this spline.
     *
     * @return Order of spline: 2 = linear, 4 = cubic, 6 = quintic, 8 = heptic.
     */
    int getOrder() const;
    /**
     * Get the half order of this spline.
     *
     * @return Half order of spline: 1 = linear, 2 = cubic, 3 = quintic, 4 = heptic.
     */
    int getHalfOrder() const;
    /**
     * Get size or number of independent data points (or number of coefficients)
     * used to construct the spline.
     *
     * @return Number of data points (or number of coefficients).
     */
    int getSize() const;
    const Array<double>& getX() const;
    /**
     * Get the array of independent variables used to construct the spline.
     *
     * @return Pointer to the independent variable data points.
     */
    virtual const double* getXValues() const;
    /**
     * Get the array of dependent variables used to construct the spline.
     *
     * @return Pointer to the dependent variable data points.
     */
    virtual const double* getYValues() const;
    /**
     * Get the array of coefficients for the spline.
     *
     * @return Pointer to the coefficients.
     */
    const Array<double>& getCoefficients() const;
    virtual int getNumberOfPoints() const { return _x.getSize(); }
    /**
     * Get the array of independent variables used to construct the spline.
     *
     * @return Reference to the independent variable data points.
     */
    virtual double getX(int aIndex) const;
    virtual double getY(int aIndex) const;
    virtual double getZ(int aIndex) const { return 0.0; }
    virtual void setX(int aIndex, double aValue);
    virtual void setY(int aIndex, double aValue);
    /**
     * Get the minimum value of the independent variable.
     *
     * @return Minimum value of the independent variable.
     */
    virtual double getMinX() const;
    /**
     * Get the maximum value of the independent variable.
     *
     * @return Maximum value of the independent variable.
     */
    virtual double getMaxX() const;
    virtual bool deletePoint(int aIndex);
    virtual bool deletePoints(const Array<int>& indices);
    virtual int addPoint(double aX, double aY);
    SimTK::Function* createSimTKFunction() const override;

    //--------------------------------------------------------------------------
    // EVALUATION
    //--------------------------------------------------------------------------

//=============================================================================
};  // END class GCVSpline

}; //namespace
//=============================================================================
//=============================================================================

#endif  // OPENSIM_GCV_SPLINE_H_
