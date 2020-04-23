/* -------------------------------------------------------------------------- *
 *                  OpenSim:  PiecewiseConstantFunction.cpp                   *
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
#include "PiecewiseConstantFunction.h"
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
PiecewiseConstantFunction::~PiecewiseConstantFunction()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
PiecewiseConstantFunction::PiecewiseConstantFunction() :
       _x(_propX.getValueDblArray()),
       _y(_propY.getValueDblArray())
{
       setNull();
}
//_____________________________________________________________________________
/**
 */
PiecewiseConstantFunction::PiecewiseConstantFunction(int aN,const double *aX,const double *aY,
       const string &aName) :
       _x(_propX.getValueDblArray()),
       _y(_propY.getValueDblArray())
{
       setNull();

       // OBJECT TYPE AND NAME
       setName(aName);

       // NUMBER OF DATA POINTS
       if(aN < 2)
       {
           log_error("PiecewiseConstantFunction: there must be 2 or more "
                     "data points.");
           return;
       }

       // CHECK DATA
       if((aX==NULL)||(aY==NULL))
       {
           log_error("PiecewiseConstantFunction: NULL arrays for data points "
                     "encountered.");
           return;
       }

       // INDEPENDENT VALUES (KNOT SEQUENCE)
       _x.setSize(0);
       _x.append(aN,aX);

       _y.setSize(0);
       _y.append(aN,aY);
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 * All data members of the specified function are copied.
 *
 * @param aFunction PiecewiseConstantFunction object to be copied.
 */
PiecewiseConstantFunction::PiecewiseConstantFunction(const PiecewiseConstantFunction &aFunction) :
       Function(aFunction),
       _x(_propX.getValueDblArray()),
       _y(_propY.getValueDblArray())
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
void PiecewiseConstantFunction::setNull()
{
       setupProperties();
}
//_____________________________________________________________________________
/**
 * Set up the serialized member variables.  This involves both generating
 * the properties and connecting them to the local pointers used to access
 * the serialized member variables.
 */
void PiecewiseConstantFunction::setupProperties()
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
void PiecewiseConstantFunction::setEqual(const PiecewiseConstantFunction &aFunction)
{
       setNull();

       // CHECK ARRAY SIZES
       if(aFunction.getSize()<=0) return;

       // ALLOCATE ARRAYS
       _x = aFunction._x;
       _y = aFunction._y;
}
//_____________________________________________________________________________
/**
 * Initialize the function with X and Y values.
 *
 * @param aN the number of X and Y values
 * @param aXValues the X values
 * @param aYValues the Y values
 */
void PiecewiseConstantFunction::init(Function* aFunction)
{
       if (aFunction == NULL)
             return;

       PiecewiseConstantFunction* sf = dynamic_cast<PiecewiseConstantFunction*>(aFunction);
       if (sf != NULL) {
             setEqual(*sf);
       } else {
             XYFunctionInterface xyFunc(aFunction);
             if (xyFunc.getNumberOfPoints() == 0) {
                  // A PiecewiseConstantFunction must have at least 2 data points.
                  // If aFunction is a Constant, use its Y value for both data points.
                  // If it is not, make up two data points.
                  double x[2] = {0.0, 1.0}, y[2];
                  Constant* cons = dynamic_cast<Constant*>(aFunction);
                  if (cons != NULL) {
                      y[0] = y[1] = cons->calcValue(SimTK::Vector(1, 0.));
                  } else {
                      y[0] = y[1] = 1.0;
                  }
                  *this = PiecewiseConstantFunction(2, x, y);
             } else if (xyFunc.getNumberOfPoints() == 1) {
                  double x[2], y[2];
                  x[0] = xyFunc.getXValues()[0];
                  x[1] = x[0] + 1.0;
                  y[0] = y[1] = xyFunc.getYValues()[0];
                  *this = PiecewiseConstantFunction(2, x, y);
             } else {
                  *this = PiecewiseConstantFunction(xyFunc.getNumberOfPoints(),
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
PiecewiseConstantFunction& PiecewiseConstantFunction::operator=(const PiecewiseConstantFunction &aFunction)
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
int PiecewiseConstantFunction::getSize() const
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
const Array<double>& PiecewiseConstantFunction::getX() const
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
const Array<double>& PiecewiseConstantFunction::getY() const
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
const double* PiecewiseConstantFunction::getXValues() const
{
       return(&_x[0]);
}
//_____________________________________________________________________________
/**
 * Get the array of dependent variables used to construct the function.
 *
 * @return Pointer to the dependent variable data points.
 */
const double* PiecewiseConstantFunction::getYValues() const
{
       return(&_y[0]);
}


//-----------------------------------------------------------------------------

//=============================================================================
// EVALUATION
//=============================================================================
/**
 * Evaluates total first derivative
 */
double PiecewiseConstantFunction::
evaluateTotalFirstDerivative(double aX,double aDxdt) const
{
       return 0.0;
}

/**
 * Evaluates total second derivative
 */
double PiecewiseConstantFunction::
evaluateTotalSecondDerivative(double aX,double aDxdt,double aD2xdt2) const
{
       return 0.0;
}

double PiecewiseConstantFunction::getX(int aIndex) const
{
       if (aIndex >= 0 && aIndex < _x.getSize())
             return _x.get(aIndex);
       else {
             throw Exception("PiecewiseConstantFunction::getX(): index out of bounds.");
             return 0.0;
       }
}

double PiecewiseConstantFunction::getY(int aIndex) const
{
       if (aIndex >= 0 && aIndex < _y.getSize())
             return _y.get(aIndex);
       else {
             throw Exception("PiecewiseConstantFunction::getY(): index out of bounds.");
             return 0.0;
       }
}

void PiecewiseConstantFunction::setX(int aIndex, double aValue)
{
       if (aIndex >= 0 && aIndex < _x.getSize()) {
             _x[aIndex] = aValue;
       } else {
             throw Exception("PiecewiseConstantFunction::setX(): index out of bounds.");
       }
}

void PiecewiseConstantFunction::setY(int aIndex, double aValue)
{
       if (aIndex >= 0 && aIndex < _y.getSize()) {
             _y[aIndex] = aValue;
       } else {
             throw Exception("PiecewiseConstantFunction::setY(): index out of bounds.");
       }
}

bool PiecewiseConstantFunction::deletePoint(int aIndex)
{
       if (_x.getSize() > 1 && _y.getSize() > 1 &&
              aIndex < _x.getSize() && aIndex < _y.getSize()) {
          _x.remove(aIndex);
          _y.remove(aIndex);
             return true;
       }

       return false;
}

bool PiecewiseConstantFunction::deletePoints(const Array<int>& indices)
{
       bool pointsDeleted = false;
       int numPointsLeft = _x.getSize() - indices.getSize();

       if (numPointsLeft >= 1) {
             // Assume the indices are sorted highest to lowest
             for (int i=0; i<indices.getSize(); i++) {
                  int index = indices.get(i);
                  if (index >= 0 && index < _x.getSize()) {
                _x.remove(index);
                _y.remove(index);
                      pointsDeleted = true;
                  }
             }
       }

   return pointsDeleted;
}

int PiecewiseConstantFunction::addPoint(double aX, double aY)
{
       int i=0;
       for (i=0; i<_x.getSize(); i++)
             if (_x[i] > aX)
                  break;

       _x.insert(i, aX);
       _y.insert(i, aY);

       return i;
}

double PiecewiseConstantFunction::calcValue(const Vector& x) const
{
    int n = _x.getSize();
    double aX = x[0];

    if (aX < _x[0] || EQUAL_WITHIN_ERROR(aX,_x[0]))
        return _y[0];
    if (aX > _x[n-1] || EQUAL_WITHIN_ERROR(aX,_x[n-1]))
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

    return _y[k];
}

double PiecewiseConstantFunction::calcDerivative(const std::vector<int>& derivComponents, const Vector& x) const
{
    return 0.0;
}

int PiecewiseConstantFunction::getArgumentSize() const
{
    return 1;
}

int PiecewiseConstantFunction::getMaxDerivativeOrder() const
{
    return std::numeric_limits<int>::max();
}

SimTK::Function* PiecewiseConstantFunction::createSimTKFunction() const {
    return new FunctionAdapter(*this);
}

