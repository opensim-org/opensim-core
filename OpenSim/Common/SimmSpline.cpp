/* -------------------------------------------------------------------------- *
 *                          OpenSim:  SimmSpline.cpp                          *
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
#include "SimmSpline.h"
#include "Constant.h"
#include "SimmMacros.h"
#include "XYFunctionInterface.h"
#include "FunctionAdapter.h"


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
SimmSpline::~SimmSpline()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SimmSpline::SimmSpline() :
    _x(_propX.getValueDblArray()),
    _y(_propY.getValueDblArray()),
    _b(0.0), _c(0.0), _d(0.0)
{
    setNull();
}
//_____________________________________________________________________________
/**
 */
SimmSpline::SimmSpline(int aN,const double *aX,const double *aY,
    const string &aName) :
    _x(_propX.getValueDblArray()),
    _y(_propY.getValueDblArray()),
    _b(0.0), _c(0.0), _d(0.0)
{
    setNull();

    // OBJECT TYPE AND NAME
    setName(aName);

    // NUMBER OF DATA POINTS
    if(aN < 2)
    {
        log_error("SimmSpline: there must be 2 or more data points.");
        return;
    }

    // CHECK DATA
    if((aX==NULL)||(aY==NULL))
    {
        log_error("SimmSpline: NULL arrays for data points encountered.");
        return;
    }

    // INDEPENDENT VALUES (KNOT SEQUENCE)
    _x.setSize(0);
    _x.append(aN,aX);

    _y.setSize(0);
    _y.append(aN,aY);

    // FIT THE SPLINE
    calcCoefficients();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 * All data members of the specified spline are copied.
 *
 * @param aSpline SimmSpline object to be copied.
 */
SimmSpline::SimmSpline(const SimmSpline &aSpline) :
    Function(aSpline),
    _x(_propX.getValueDblArray()),
    _y(_propY.getValueDblArray()),
    _b(0.0), _c(0.0), _d(0.0)
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
void SimmSpline::setNull()
{
    setupProperties();
}
//_____________________________________________________________________________
/**
 * Set up the serialized member variables.  This involves both generating
 * the properties and connecting them to the local pointers used to access
 * the serialized member variables.
 */
void SimmSpline::setupProperties()
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
void SimmSpline::setEqual(const SimmSpline &aSpline)
{
    setNull();

    // CHECK ARRAY SIZES
    if(aSpline.getSize()<=0) return;

    // ALLOCATE ARRAYS
    _x = aSpline._x;
    _y = aSpline._y;
    _b = aSpline._b;
    _c = aSpline._c;
    _d = aSpline._d;
}
//_____________________________________________________________________________
/**
 * Initialize the spline with X and Y values.
 *
 * @param aN the number of X and Y values
 * @param aXValues the X values
 * @param aYValues the Y values
 */
void SimmSpline::init(Function* aFunction)
{
    if (aFunction == NULL)
        return;

    SimmSpline* ncs = dynamic_cast<SimmSpline*>(aFunction);
    if (ncs != NULL) {
        setEqual(*ncs);
    } else {
        XYFunctionInterface xyFunc(aFunction);
        if (xyFunc.getNumberOfPoints() == 0) {
            // A SimmSpline must have at least 2 data points.
            // If aFunction is a Constant, use its Y value for both data points.
            // If it is not, make up two data points.
            double x[2] = {0.0, 1.0}, y[2];
            Constant* cons = dynamic_cast<Constant*>(aFunction);
            if (cons != NULL) {
                y[0] = y[1] = cons->calcValue(SimTK::Vector(0));
            } else {
                y[0] = y[1] = 1.0;
            }
            *this = SimmSpline(2, x, y);
        } else if (xyFunc.getNumberOfPoints() == 1) {
            double x[2], y[2];
            x[0] = xyFunc.getXValues()[0];
            x[1] = x[0] + 1.0;
            y[0] = y[1] = xyFunc.getYValues()[0];
            *this = SimmSpline(2, x, y);
        } else {
            *this = SimmSpline(xyFunc.getNumberOfPoints(),
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
SimmSpline& SimmSpline::operator=(const SimmSpline &aSpline)
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
// NUMBER OF DATA POINTS (N)
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get size or number of independent data points (or number of coefficients)
 * used to construct the spline.
 *
 * @return Number of data points (or number of coefficients).
 */
int SimmSpline::getSize() const
{
    return(_x.getSize());
}

//-----------------------------------------------------------------------------
// X AND COEFFICIENTS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the array of independent variables used to construct the spline.
 * For the number of independent variable data points use getN().
 *
 * @return Pointer to the independent variable data points.
 * @see getN();
 */
const Array<double>& SimmSpline::getX() const
{
    return(_x);
}
//_____________________________________________________________________________
/**
 * Get the array of Y values for the spline.
 * For the number of Y values use getNX().
 *
 * @return Pointer to the coefficients.
 * @see getCoefficients();
 */
const Array<double>& SimmSpline::getY() const
{
    return(_y);
}
//_____________________________________________________________________________
/**
 * Get the array of independent variables used to construct the spline.
 * For the number of independent variable data points use getN().
 *
 * @return Pointer to the independent variable data points.
 * @see getN();
 */
const double* SimmSpline::getXValues() const
{
    return(&_x[0]);
}
//_____________________________________________________________________________
/**
 * Get the array of dependent variables used to construct the spline.
 *
 * @return Pointer to the independent variable data points.
 */
const double* SimmSpline::getYValues() const
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
void SimmSpline::updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
    Function::updateFromXMLNode(aNode, versionNumber);
    calcCoefficients();
}   

//=============================================================================
// EVALUATION
//=============================================================================
void SimmSpline::calcCoefficients()
{
   int n = _x.getSize();
    int nm1, nm2, i, j;
   double t;

   if (n < 2)
      return;

   _b.setSize(n);
   _c.setSize(n);
   _d.setSize(n);

   if (n == 2)
   {
      t = MAX(TINY_NUMBER,_x[1]-_x[0]);
      _b[0] = _b[1] = (_y[1]-_y[0])/t;
      _c[0] = _c[1] = 0.0;
      _d[0] = _d[1] = 0.0;
      return;
   }

   nm1 = n - 1;
   nm2 = n - 2;

   /* Set up tridiagonal system:
    * b = diagonal, d = offdiagonal, c = right-hand side
    */

   _d[0] = MAX(TINY_NUMBER,_x[1] - _x[0]);
   _c[1] = (_y[1]-_y[0])/_d[0];
   for (i=1; i<nm1; i++)
   {
      _d[i] = MAX(TINY_NUMBER,_x[i+1] - _x[i]);
      _b[i] = 2.0*(_d[i-1]+_d[i]);
      _c[i+1] = (_y[i+1]-_y[i])/_d[i];
      _c[i] = _c[i+1] - _c[i];
   }

   /* End conditions. Third derivatives at x[0] and x[n-1]
    * are obtained from divided differences.
    */

   _b[0] = -_d[0];
   _b[nm1] = -_d[nm2];
   _c[0] = 0.0;
   _c[nm1] = 0.0;

   if (n > 3)
   {
      double d1, d2, d3, d20, d30, d31;

      d31 = MAX(TINY_NUMBER,_x[3] - _x[1]);
      d20 = MAX(TINY_NUMBER,_x[2] - _x[0]);
      d1 = MAX(TINY_NUMBER,_x[nm1]-_x[n-3]);
      d2 = MAX(TINY_NUMBER,_x[nm2]-_x[n-4]);
      d30 = MAX(TINY_NUMBER,_x[3] - _x[0]);
      d3 = MAX(TINY_NUMBER,_x[nm1]-_x[n-4]);
      _c[0] = _c[2]/d31 - _c[1]/d20;
      _c[nm1] = _c[nm2]/d1 - _c[n-3]/d2;
      _c[0] = _c[0]*_d[0]*_d[0]/d30;
      _c[nm1] = -_c[nm1]*_d[nm2]*_d[nm2]/d3;
   }

   /* Forward elimination */

   for (i=1; i<n; i++)
   {
      t = _d[i-1]/_b[i-1];
      _b[i] -= t*_d[i-1];
      _c[i] -= t*_c[i-1];
   }

   /* Back substitution */

   _c[nm1] /= _b[nm1];
   for (j=0; j<nm1; j++)
   {
      i = nm2 - j;
      _c[i] = (_c[i]-_d[i]*_c[i+1])/_b[i];
   }

   /* compute polynomial coefficients */

   _b[nm1] = (_y[nm1]-_y[nm2])/_d[nm2] +
               _d[nm2]*(_c[nm2]+2.0*_c[nm1]);
   for (i=0; i<nm1; i++)
   {
      _b[i] = (_y[i+1]-_y[i])/_d[i] - _d[i]*(_c[i+1]+2.0*_c[i]);
      _d[i] = (_c[i+1]-_c[i])/_d[i];
      _c[i] *= 3.0;
   }
   _c[nm1] *= 3.0;
   _d[nm1] = _d[nm2];

}

double SimmSpline::getX(int aIndex) const
{
    if (aIndex >= 0 && aIndex < _x.getSize())
        return _x.get(aIndex);
    else {
        throw Exception("SimmSpline::getX(): index out of bounds.");
        return 0.0;
    }
}

double SimmSpline::getY(int aIndex) const
{
    if (aIndex >= 0 && aIndex < _y.getSize())
        return _y.get(aIndex);
    else {
        throw Exception("SimmSpline::getY(): index out of bounds.");
        return 0.0;
    }
}

void SimmSpline::setX(int aIndex, double aValue)
{
    if (aIndex >= 0 && aIndex < _x.getSize()) {
        _x[aIndex] = aValue;
       calcCoefficients();
    } else {
        throw Exception("SimmSpline::setX(): index out of bounds.");
    }
}

void SimmSpline::setY(int aIndex, double aValue)
{
    if (aIndex >= 0 && aIndex < _y.getSize()) {
        _y[aIndex] = aValue;
       calcCoefficients();
    } else {
        throw Exception("SimmSpline::setY(): index out of bounds.");
    }
}

bool SimmSpline::deletePoint(int aIndex)
{
    if (_x.getSize() > 2 && _y.getSize() > 2 &&
         aIndex < _x.getSize() && aIndex < _y.getSize()) {
       _x.remove(aIndex);
       _y.remove(aIndex);

       // Recalculate the coefficients
       calcCoefficients();
        return true;
   }

   return false;
}

bool SimmSpline::deletePoints(const Array<int>& indices)
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

int SimmSpline::addPoint(double aX, double aY)
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

double SimmSpline::calcValue(const Vector& x) const
{
    // NOT A NUMBER
    if(!_y.getSize()) return(SimTK::NaN);
    if(!_b.getSize()) return(SimTK::NaN);
    if(!_c.getSize()) return(SimTK::NaN);
    if(!_d.getSize()) return(SimTK::NaN);

    int i, j, k;
    double dx;

    int n = _x.getSize();
    double aX = x[0];

   /* Check if the abscissa is out of range of the function. If it is,
    * then use the slope of the function at the appropriate end point to
    * extrapolate. You do this rather than printing an error because the
    * assumption is that this will only occur in relatively harmless
    * situations (like a motion file that contains an out-of-range coordinate
    * value). The rest of the SIMM code has many checks to clamp a coordinate
    * value within its range of motion, so if you make it to this function
    * and the coordinate is still out of range, deal with it quietly.
    */

   if (aX < _x[0])
       return _y[0] + (aX - _x[0])*_b[0];
   else if (aX > _x[n-1])
       return _y[n-1] + (aX - _x[n-1])*_b[n-1];

   /* Check to see if the abscissa is close to one of the end points
    * (the binary search method doesn't work well if you are at one of the
    * end points.
    */
   if (EQUAL_WITHIN_ERROR(aX,_x[0]))
       return _y[0];
   else if (EQUAL_WITHIN_ERROR(aX,_x[n-1]))
       return _y[n-1];

    if (n < 3)
    {
        /* If there are only 2 function points, then set k to zero
         * (you've already checked to see if the abscissa is out of
         * range or equal to one of the endpoints).
         */
        k = 0;
    }
    else
    {
        /* Do a binary search to find which two points the abscissa is between. */
        i = 0;
        j = n;
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
    }

   dx = aX - _x[k];
   return _y[k] + dx*(_b[k] + dx*(_c[k] + dx*_d[k]));
}

double SimmSpline::calcDerivative(const std::vector<int>& derivComponents, const Vector& x) const
{
    // NOT A NUMBER
    if(!_y.getSize()) return(SimTK::NaN);
    if(!_b.getSize()) return(SimTK::NaN);
    if(!_c.getSize()) return(SimTK::NaN);
    if(!_d.getSize()) return(SimTK::NaN);

    int i, j, k;
    double dx;

    int n = _x.getSize();
    double aX = x[0];
    int aDerivOrder = (int)derivComponents.size();
    if (aDerivOrder < 1 || aDerivOrder > 2)
        throw Exception("SimmSpline::calcDerivative(): derivative order must be 1 or 2.");

   /* Check if the abscissa is out of range of the function. If it is,
    * then use the slope of the function at the appropriate end point to
    * extrapolate. You do this rather than printing an error because the
    * assumption is that this will only occur in relatively harmless
    * situations (like a motion file that contains an out-of-range coordinate
    * value). The rest of the SIMM code has many checks to clamp a coordinate
    * value within its range of motion, so if you make it to this function
    * and the coordinate is still out of range, deal with it quietly.
    */

   if (aX < _x[0])
   {
      if (aDerivOrder == 1)
         return _b[0];
      else
         return 0;
   }
   else if (aX > _x[n-1])
   {
      if (aDerivOrder == 1)
         return _b[n-1];
      else
         return 0;
   }

   /* Check to see if the abscissa is close to one of the end points
    * (the binary search method doesn't work well if you are at one of the
    * end points.
    */
   if (EQUAL_WITHIN_ERROR(aX,_x[0]))
   {
      if (aDerivOrder == 1)
         return _b[0];
      else
         return 2.0*_c[0];
   }
   else if (EQUAL_WITHIN_ERROR(aX,_x[n-1]))
   {
      if (aDerivOrder == 1)
         return _b[n-1];
      else
         return 2.0*_c[n-1];
   }

    if (n < 3)
    {
        /* If there are only 2 function points, then set k to zero
         * (you've already checked to see if the abscissa is out of
         * range or equal to one of the endpoints).
         */
        k = 0;
    }
    else
    {
        /* Do a binary search to find which two points the abscissa is between. */
        i = 0;
        j = n;
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
    }

   dx = aX - _x[k];

   if (aDerivOrder == 1)
      return (_b[k] + dx*(2.0*_c[k] + 3.0*dx*_d[k]));

   else
      return (2.0*_c[k] + 6.0*dx*_d[k]);
}

int SimmSpline::getArgumentSize() const
{
    return 1;
}

int SimmSpline::getMaxDerivativeOrder() const
{
    return 2;
}

SimTK::Function* SimmSpline::createSimTKFunction() const {
    return new FunctionAdapter(*this);
}
