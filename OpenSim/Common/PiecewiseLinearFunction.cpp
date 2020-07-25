/* -------------------------------------------------------------------------- *
 *                   OpenSim:  PiecewiseLinearFunction.cpp                    *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Peter Loan                                                      *
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


// C++ INCLUDES
#include "PiecewiseLinearFunction.h"
#include "Constant.h"
#include "FunctionAdapter.h"
#include "SimmMacros.h"
#include "XYFunctionInterface.h"


using namespace OpenSim;
using namespace std;
using SimTK::Vector;


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
PiecewiseLinearFunction::~PiecewiseLinearFunction()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
PiecewiseLinearFunction::PiecewiseLinearFunction() :
    _x(_propX.getValueDblArray()),
    _y(_propY.getValueDblArray())
{
    setNull();
}
//_____________________________________________________________________________
/**
 */
PiecewiseLinearFunction::PiecewiseLinearFunction(int aN,const double *aX,const double *aY,
    const string &aName) :
    _x(_propX.getValueDblArray()),
    _y(_propY.getValueDblArray()),
   _b(0.0)
{
    setNull();

    // OBJECT TYPE AND NAME
    setName(aName);

    // NUMBER OF DATA POINTS
    OPENSIM_THROW_IF_FRMOBJ(aN < 2, Exception,
            "PiecewiseLinearFunction: there must be 2 or more data "
            "points, but got {} data points.",
            aN);

    // CHECK DATA
    OPENSIM_THROW_IF_FRMOBJ(aX == nullptr || aY == nullptr, Exception,
        "x and/or y data is null.");

    for (int i = 1; i < aN; ++i) {
        OPENSIM_THROW_IF_FRMOBJ(aX[i] < aX[i - 1], Exception,
                "Expected independent variable to be non-decreasing, but x[{}] "
                "= {} is less than x[{}] = {}",
                i, aX[i], i - 1, aX[i - 1]);
    }

    // INDEPENDENT VALUES (KNOT SEQUENCE)
    _x.setSize(0);
    _x.append(aN,aX);

    _y.setSize(0);
    _y.append(aN,aY);

    // Calculate the slope coefficients
    calcCoefficients();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 * All data members of the specified function are copied.
 *
 * @param aFunction PiecewiseLinearFunction object to be copied.
 */
PiecewiseLinearFunction::PiecewiseLinearFunction(const PiecewiseLinearFunction &aFunction) :
    Function(aFunction),
    _x(_propX.getValueDblArray()),
    _y(_propY.getValueDblArray()),
   _b(0.0)
{
    setEqual(aFunction);
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set all member variables to their NULL or default values.
 */
void PiecewiseLinearFunction::setNull()
{
    setupProperties();
}
//_____________________________________________________________________________
/**
 * Set up the serialized member variables.  This involves both generating
 * the properties and connecting them to the local pointers used to access
 * the serialized member variables.
 */
void PiecewiseLinearFunction::setupProperties()
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
void PiecewiseLinearFunction::setEqual(const PiecewiseLinearFunction &aFunction)
{
    setNull();

    // CHECK ARRAY SIZES
    if(aFunction.getSize()<=0) return;

    // ALLOCATE ARRAYS
    _x = aFunction._x;
    _y = aFunction._y;
    _b = aFunction._b;
}
//_____________________________________________________________________________
/**
 * Initialize the function with X and Y values.
 *
 * @param aN the number of X and Y values
 * @param aXValues the X values
 * @param aYValues the Y values
 */
void PiecewiseLinearFunction::init(Function* aFunction)
{
    if (aFunction == NULL)
        return;

    PiecewiseLinearFunction* lf = dynamic_cast<PiecewiseLinearFunction*>(aFunction);
    if (lf != NULL) {
        setEqual(*lf);
    } else {
        XYFunctionInterface xyFunc(aFunction);
        if (xyFunc.getNumberOfPoints() == 0) {
            // A PiecewiseLinearFunction must have at least 2 data points.
            // If aFunction is a Constant, use its Y value for both data points.
            // If it is not, make up two data points.
            double x[2] = {0.0, 1.0}, y[2];
            Constant* cons = dynamic_cast<Constant*>(aFunction);
            if (cons != NULL) {
                y[0] = y[1] = cons->calcValue(SimTK::Vector(0));
            } else {
                y[0] = y[1] = 1.0;
            }
            *this = PiecewiseLinearFunction(2, x, y);
        } else if (xyFunc.getNumberOfPoints() == 1) {
            double x[2], y[2];
            x[0] = xyFunc.getXValues()[0];
            x[1] = x[0] + 1.0;
            y[0] = y[1] = xyFunc.getYValues()[0];
            *this = PiecewiseLinearFunction(2, x, y);
        } else {
            *this = PiecewiseLinearFunction(xyFunc.getNumberOfPoints(),
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
PiecewiseLinearFunction& PiecewiseLinearFunction::operator=(const PiecewiseLinearFunction &aFunction)
{
    // BASE CLASS
    Function::operator=(aFunction);

    // DATA
    setEqual(aFunction);

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
 * used to construct the function.
 *
 * @return Number of data points (or number of coefficients).
 */
int PiecewiseLinearFunction::getSize() const
{
    return(_x.getSize());
}

//-----------------------------------------------------------------------------
// X AND COEFFICIENTS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the array of independent variables used to construct the function.
 * For the number of independent variable data points use getN().
 *
 * @return Pointer to the independent variable data points.
 * @see getN();
 */
const Array<double>& PiecewiseLinearFunction::getX() const
{
    return(_x);
}
//_____________________________________________________________________________
/**
 * Get the array of Y values for the function.
 * For the number of Y values use getNX().
 *
 * @return Pointer to the coefficients.
 * @see getCoefficients();
 */
const Array<double>& PiecewiseLinearFunction::getY() const
{
    return(_y);
}
//_____________________________________________________________________________
/**
 * Get the array of independent variables used to construct the function.
 * For the number of independent variable data points use getN().
 *
 * @return Pointer to the independent variable data points.
 * @see getN();
 */
const double* PiecewiseLinearFunction::getXValues() const
{
    return(&_x[0]);
}
//_____________________________________________________________________________
/**
 * Get the array of dependent variables used to construct the function.
 *
 * @return Pointer to the dependent variable data points.
 */
const double* PiecewiseLinearFunction::getYValues() const
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
void PiecewiseLinearFunction::updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
    Function::updateFromXMLNode(aNode, versionNumber);
    calcCoefficients();
}   

double PiecewiseLinearFunction::getX(int aIndex) const
{
    if (aIndex >= 0 && aIndex < _x.getSize())
        return _x.get(aIndex);
    else {
        throw Exception("PiecewiseLinearFunction::getX(): index out of bounds.");
        return 0.0;
    }
}

double PiecewiseLinearFunction::getY(int aIndex) const
{
    if (aIndex >= 0 && aIndex < _y.getSize())
        return _y.get(aIndex);
    else {
        throw Exception("PiecewiseLinearFunction::getY(): index out of bounds.");
        return 0.0;
    }
}

void PiecewiseLinearFunction::setX(int aIndex, double aValue)
{
    if (aIndex >= 0 && aIndex < _x.getSize()) {
        _x[aIndex] = aValue;
        calcCoefficients();
    } else {
        throw Exception("PiecewiseLinearFunction::setX(): index out of bounds.");
    }
}

void PiecewiseLinearFunction::setY(int aIndex, double aValue)
{
    if (aIndex >= 0 && aIndex < _y.getSize()) {
        _y[aIndex] = aValue;
        calcCoefficients();
    } else {
        throw Exception("PiecewiseLinearFunction::setY(): index out of bounds.");
    }
}

bool PiecewiseLinearFunction::deletePoint(int aIndex)
{
    if (_x.getSize() > 2 && _y.getSize() > 2 &&
         aIndex < _x.getSize() && aIndex < _y.getSize()) {
       _x.remove(aIndex);
       _y.remove(aIndex);

       // Recalculate the slopes
      calcCoefficients();
        return true;
    }

   return false;
}

bool PiecewiseLinearFunction::deletePoints(const Array<int>& indices)
{
    bool pointsDeleted = false;
    int numPointsLeft = _x.getSize() - indices.getSize();

    if (numPointsLeft >= 2) {
        // Assume the indices are sorted highest to lowest
        for (int i=0; i<indices.getSize(); i++) {
            int index = indices.get(i);
            if (index >= 0 && index < _x.getSize()) {
             _x.remove(index);
             _y.remove(index);
                pointsDeleted = true;
            }
        }
        if (pointsDeleted)
            calcCoefficients();
    }

   return pointsDeleted;
}

int PiecewiseLinearFunction::addPoint(double aX, double aY)
{
    int i=0;
    for (i=0; i<_x.getSize(); i++)
        if (_x[i] > aX)
            break;

    _x.insert(i, aX);
    _y.insert(i, aY);

    // Recalculate the slopes
    calcCoefficients();

    return i;
}

//=============================================================================
// EVALUATION
//=============================================================================
void PiecewiseLinearFunction::calcCoefficients()
{
   int n = _x.getSize();

   if (n < 2)
      return;

   _b.setSize(n);

    for (int i=0; i<n-1; i++) {
        double range = MAX(TINY_NUMBER, _x[i+1] - _x[i]);
        _b[i] = (_y[i+1] - _y[i]) / range;
    }
    _b[n-1] = _b[n-2];
}

double PiecewiseLinearFunction::calcValue(const Vector& x) const
{
    int n = _x.getSize();
    double aX = x[0];

    if (aX < _x[0])
        return _y[0] + (aX - _x[0]) * _b[0];
    else if (aX > _x[n-1])
        return _y[n-1] + (aX - _x[n-1]) * _b[n-1];

    /* Check to see if the abscissa is close to one of the end points
     * (the binary search method doesn't work well if you are at one of the
     * end points.
     */
    if (EQUAL_WITHIN_ERROR(aX, _x[0]))
        return _y[0];
    else if (EQUAL_WITHIN_ERROR(aX,_x[n-1]))
        return _y[n-1];

    // Do a binary search to find which two points the abscissa is between.
    int k, i = 0;
    int j = n;
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

    return _y[k] + (aX - _x[k]) * _b[k];
}

double PiecewiseLinearFunction::calcDerivative(const std::vector<int>& derivComponents, const Vector& x) const
{
    if (derivComponents.size() == 0)
        return SimTK::NaN;
    if (derivComponents.size() > 1)
        return 0.0;

    int n = _x.getSize();
    double aX = x[0];

    if (aX < _x[0]) {
        return _b[0];
    } else if (aX > _x[n-1]) {
        return _b[n-1];
    }

    /* Check to see if the abscissa is close to one of the end points
     * (the binary search method doesn't work well if you are at one of the
     * end points.
     */
    if (EQUAL_WITHIN_ERROR(aX, _x[0])) {
        return _b[0];
    } else if (EQUAL_WITHIN_ERROR(aX,_x[n-1])) {
        return _b[n-1];
    }

    // Do a binary search to find which two points the abscissa is between.
    int k, i = 0;
    int j = n;
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

    return _b[k];
}

int PiecewiseLinearFunction::getArgumentSize() const
{
    return 1;
}

int PiecewiseLinearFunction::getMaxDerivativeOrder() const
{
    return std::numeric_limits<int>::max();
}

SimTK::Function* PiecewiseLinearFunction::createSimTKFunction() const {
    return new FunctionAdapter(*this);
}
