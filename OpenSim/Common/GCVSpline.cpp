/* -------------------------------------------------------------------------- *
 *                          OpenSim:  GCVSpline.cpp                           *
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

/*  
 * Author:  Frank C. Anderson
 
 */


// C++ INCLUDES
#include "GCVSpline.h"
#include "Constant.h"
#include "gcvspl.h"
#include "XYFunctionInterface.h"



using namespace OpenSim;
using namespace std;
using SimTK::Vector_;
using SimTK::Vector;
using SimTK::Vec;


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
    _y(_propY.getValueDblArray()),
    _workDeriv(1)
{
    setNull();
}
//_____________________________________________________________________________
GCVSpline::
GCVSpline(int aDegree,int aN,const double *aX,const double *aF,
    const string &aName,double aErrorVariance) :
    _halfOrder(_propHalfOrder.getValueInt()),
    _errorVariance(_propErrorVariance.getValueDbl()),
    _x(_propX.getValueDblArray()),
    _weights(_propWeights.getValueDblArray()),
    _coefficients(_propCoefficients.getValueDblArray()),
    _y(_propY.getValueDblArray()),
    _workDeriv(1)

{
    setNull();

    // OBJECT TYPE AND NAME
    setName(aName);

    // DEGREE
    setDegree(aDegree);

    // NUMBER OF DATA POINTS
    if(aN < getOrder()) {
        log_error("GCVSpline: there must be {} or more data points.",
            getOrder());
        return;
    }

    // CHECK DATA
    if((aX==NULL)||(aF==NULL)) {
        log_error("GCVSpline: NULL arrays for data points encountered.");
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
    int sz = _weights.getSize();
    for(i=0;i<sz;i++) _weights[i] = 1.0;

    // ALLOCATE COEFFICIENTS
    _coefficients.setSize(_x.getSize());

    // FIT THE SPLINE
    _errorVariance = aErrorVariance;
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
    _y(_propY.getValueDblArray()),
    _workDeriv(1)

{
    setEqual(aSpline);
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
    setupProperties();
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

    // Y- DEPENDENT VARIABLES
    _propY.setName("y");
    Array<double> y(0.0);
    _propY.setValue(y);
    _propertySet.append( &_propY );

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

    // CHECK ARRAY SIZES
    if(aSpline.getSize()<=0) return;
    if(_halfOrder<=0) return;

    // ALLOCATE ARRAYS
    _x = aSpline._x;
    _y = aSpline._y;
    _weights = aSpline._weights;
    _coefficients = aSpline._coefficients;
}

//-----------------------------------------------------------------------------
// UPDATE FROM XML NODE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Update this object based on its XML node.
 */
void GCVSpline::updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
    // Base class
    Function::updateFromXMLNode(aNode, versionNumber);

    // Weights may not have been specified in the XML file.
    int wSize = _weights.getSize();
    if (wSize < _x.getSize()) {
        _weights.setSize(_x.getSize()); 
        for (int i=wSize; i<_x.getSize(); i++)
            _weights[i] = 1.0;
    }

    // Coefficients may not have been specified in the XML file.
    if (_coefficients.getSize() < _x.getSize())
        _coefficients.setSize(_x.getSize());
}   

//_____________________________________________________________________________
/**
 * Initialize the spline with X and Y values.
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
    } else {
        XYFunctionInterface xyFunc(aFunction);
        if (xyFunc.getNumberOfPoints() == 0) {
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
                    y[i] = cons->calcValue(SimTK::Vector(0));
            } else {
                for (int i=0; i<order; i++)
                    y[i] = 1.0;
            }
            *this = GCVSpline(degree, order, x, y);
            delete [] x;
            delete [] y;
        } else if (xyFunc.getNumberOfPoints() < order) {
            // A GCVSpline must have at least getOrder() data points.
            // Use as many data points as aFunction has, and then fill
            // in the rest by copying the last Y value, and incrementing
            // the X value by the step between the last two real data points.
            double* x = new double[order];
            double* y = new double[order];
            double step = 1.0;
            if (xyFunc.getNumberOfPoints() >= 2)
                step = xyFunc.getXValues()[xyFunc.getNumberOfPoints()-1] -
                xyFunc.getXValues()[xyFunc.getNumberOfPoints()-2];
            for (int i=0; i<xyFunc.getNumberOfPoints(); i++) {
                x[i] = xyFunc.getXValues()[i];
                y[i] = xyFunc.getYValues()[i];
            }
            for (int i=xyFunc.getNumberOfPoints(); i<order; i++) {
                x[i] = x[i-1] + step;
                y[i] = y[i-1];
            }
            *this = GCVSpline(degree, order, x, y);
            delete [] x;
            delete [] y;
        } else {
            *this = GCVSpline(degree, xyFunc.getNumberOfPoints(),
                xyFunc.getXValues(), xyFunc.getYValues());
        }
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
void GCVSpline::
setDegree(int aDegree)
{
    _halfOrder = (aDegree + 1) / 2;

    // TOO SMALL
    if(_halfOrder<1) {
        log_warn("GCVSpline.setDegree: invalid half order {}. Setting degree = "
                 "1 (linear spline.)",
                _halfOrder);
        _halfOrder = 1;
    }

    // TOO LARGE
    if(_halfOrder>4) {
        log_warn("GCVSpline.setDegree: invalid half order {}. Setting degree = "
                 "7 (heptic spline.)",
                _halfOrder);
        _halfOrder = 4;
    }
}
//_____________________________________________________________________________
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
int GCVSpline::
getHalfOrder() const
{
    return(_halfOrder);
}

//-----------------------------------------------------------------------------
// NUMBER OF DATA POINTS (N)
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
int GCVSpline::
getSize() const
{
    return(_x.getSize());
}

//-----------------------------------------------------------------------------
// X AND COEFFICIENTS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
const Array<double>& GCVSpline::
getX() const
{
    return(_x);
}
//_____________________________________________________________________________
const double* GCVSpline::
getXValues() const
{
    return(&_x[0]);
}
//_____________________________________________________________________________
const double* GCVSpline::
getYValues() const
{
    return(&_y[0]);
}
//_____________________________________________________________________________
const Array<double>& GCVSpline::
getCoefficients() const
{
    return(_coefficients);
}



//=============================================================================
// EVALUATION
//=============================================================================
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
        resetFunction();
    } else {
        throw Exception("GCVSpline::setX(): index out of bounds.");
    }
}

void GCVSpline::
setY(int aIndex, double aValue)
{
    if (aIndex >= 0 && aIndex < _y.getSize()) {
        _y[aIndex] = aValue;
        resetFunction();
    } else {
        throw Exception("GCVSpline::setY(): index out of bounds.");
    }
}

//-----------------------------------------------------------------------------
// MIN AND MAX X
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
double GCVSpline::
getMinX() const
{
    if(getSize()<=0) return(SimTK::NaN);
    return(_x.get(0));
}
//_____________________________________________________________________________
double GCVSpline::
getMaxX() const
{
    if(getSize()<=0) return(SimTK::NaN);
    return(_x.getLast());
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
       resetFunction();
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
            // Recalculate the coefficients
            resetFunction();
        }
    }

   return pointsDeleted;
}

int GCVSpline::
addPoint(double aX, double aY)
{
    int i=0;
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

    // Recalculate the coefficients
    resetFunction();

    return i;
}

SimTK::Function* GCVSpline::createSimTKFunction() const {
    int degree = _halfOrder*2-1;
    Vector x(_x.getSize());
    Vector y(_y.getSize());
    for (int i = 0; i < x.size(); ++i)
        x[i] = _x[i];
    for (int i = 0; i < y.size(); ++i)
        y[i] = _y[i];
    SimTK::Spline* spline;
    if (_errorVariance < 0.0)
        spline = new SimTK::Spline(SimTK::SplineFitter<double>::fitFromGCV(degree, x, y).getSpline());
    else
        spline = new SimTK::Spline(SimTK::SplineFitter<double>::fitFromErrorVariance(degree, x, y, _errorVariance).getSpline());

     int sz = _coefficients.getSize();
    for (int i = 0; i < sz; ++i)
        _coefficients[i] = spline->getControlPointValues()[i];
    return spline;
}

