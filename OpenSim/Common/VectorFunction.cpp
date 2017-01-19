/* -------------------------------------------------------------------------- *
 *                        OpenSim:  VectorFunction.cpp                        *
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


// INCLUDES
#include "VectorFunction.h"



using namespace OpenSim;
using namespace std;


//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
VectorFunction::~VectorFunction()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
VectorFunction::VectorFunction() :
    _minX(0.0), _maxX(0.0)
{
    setNull();
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
VectorFunction::VectorFunction(int aNX, int aNY) :
    _minX(-std::numeric_limits<SimTK::Real>::infinity() ),
    _maxX( std::numeric_limits<SimTK::Real>::infinity() )
{
    setNull();
    setNX(aNX);
    setNY(aNY);
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aVectorFunction Function to copy.
 */
VectorFunction::VectorFunction(const VectorFunction &aVectorFunction) :
    Object(aVectorFunction),
    _minX(-std::numeric_limits<SimTK::Real>::infinity() ),
    _maxX( std::numeric_limits<SimTK::Real>::infinity() )
{
    setNull();

    // ASSIGN
    setEqual(aVectorFunction);
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set all member variables to NULL values.
 */
void VectorFunction::
setNull()
{
    _nX = 0;
    _nY = 0;
}

//_____________________________________________________________________________
/**
 * Set all member variables equal to the members of another object.
 * Note that this method is private.  It is only meant for copying the data
 * members defined in this class.  It does not, for example, make any changes
 * to data members of base classes.
 */
void VectorFunction::
setEqual(const VectorFunction &aVectorFunction)
{
    setNX(aVectorFunction.getNX());
    setNY(aVectorFunction.getNY());
    setMinX(aVectorFunction.getMinX());
    setMaxX(aVectorFunction.getMaxX());
}

//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
VectorFunction& VectorFunction::
operator=(const VectorFunction &aVectorFunction)
{
    // BASE CLASS
    Object::operator=(aVectorFunction);

    // DATA
    setEqual(aVectorFunction);

    return(*this);
}


//=============================================================================
// SET AND GET
//=============================================================================
//-----------------------------------------------------------------------------
// NUMBER OF INDEPENDENT AND DEPENDENT VARIABLES
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the number of independent variables.
 *
 * @param aNX Number of independent variables.
 */
void VectorFunction::
setNX(int aNX)
{
    _nX = aNX;
}
//_____________________________________________________________________________
/**
 * Get the number of independent variables.
 *
 * @return Number of independent variables.
 */
int VectorFunction::
getNX() const
{
    return(_nX);
}
//_____________________________________________________________________________
/**
 * Set the number of dependent variables.
 *
 * @param aNX Number of dependent variables.
 */
void VectorFunction::
setNY(int aNY)
{
    _nY = aNY;
}
//_____________________________________________________________________________
/**
 * Get the number of dependent variables.
 *
 * @return Number of dependent variables.
 */
int VectorFunction::
getNY() const
{
    return(_nY);
}
//-----------------------------------------------------------------------------
// MIN AND MAX INDEPENDENT VARIABLES
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the minimum values of the independent variables.
 *
 * @param aMinX Array of minimum values of the independent variables.
 */
void VectorFunction::
setMinX(const Array<double> &aMinX)
{
    if(aMinX.getSize()!=_nX) {
        string msg = "VectorFunction.setMinX: ERR- ";
        msg += "Array size does not match number of variables.";
        throw( Exception(msg,__FILE__,__LINE__) );
    }
    _minX = aMinX;
}
//_____________________________________________________________________________
/**
 * Get the minimum values of the independent variables.
 *
 * @return Array of minimum values of the independent variables.
 */
const Array<double>& VectorFunction::
getMinX() const
{
    return(_minX);
}
//_____________________________________________________________________________
/**
 * Set the minimum value of one of the independent variables.
 *
 * @param aXIndex Index of the independent variable value to be set.
 * @param aMinX Value of the independent variable specified by aX.
 */
void VectorFunction::
setMinX(int aXIndex, double aMinX)
{
    _minX.set(aXIndex, aMinX);
}
//_____________________________________________________________________________
/**
 * Get the minimum values of the independent variables.
 * 
 * @param aXIndex Index of the independent variable value to be set.
 * @return Array of minimum values of the independent variables.
 */
double VectorFunction::
getMinX(int aXIndex) const
{
    return(_minX.get(aXIndex));
}
//_____________________________________________________________________________
/**
 * Set the maximum values of the independent variables.
 *
 * @param aMaxX Array of maximum values of the independent variables.
 */
void VectorFunction::
setMaxX(const Array<double> &aMaxX)
{
    if(aMaxX.getSize()!=_nX) {
        string msg = "VectorFunction.setMaxX: ERR- ";
        msg += "Array size does not.match number of variables.";
        throw( Exception(msg,__FILE__,__LINE__) );
    }

    _maxX = aMaxX;}
//_____________________________________________________________________________
/**
 * Get the maximum values of the independent variables.
 *
 * @return Array of maximum values of the independent variables.
 */
const Array<double>& VectorFunction::
getMaxX() const
{
    return(_maxX);
}
//_____________________________________________________________________________
/**
 * Set the maximum value of one of the independent variables.
 *
 * @param aX Index of the independent variable value to be set.
 * @param aMaxX Value of the independent variable specified by aX.
 */
void VectorFunction::
setMaxX(int aXIndex, double aMaxX)
{
    _maxX.set(aXIndex, aMaxX);
}
//_____________________________________________________________________________
/**
 * Get the maximum values of the independent variables.
 *
 * @param aX Index of the independent variable value to be set.
 * @return Array of maximum values of the independent variables.
 */
double VectorFunction::
getMaxX(int aXIndex) const
{
    return(_maxX.get(aXIndex));
}


//_____________________________________________________________________________
/**
 * Update the bounding box for the abscissae.
 *
 * This method should be overridden as needed by derived classes.
 */
void VectorFunction::
updateBoundingBox()
{

}

