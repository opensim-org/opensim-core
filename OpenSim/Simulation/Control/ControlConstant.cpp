/* -------------------------------------------------------------------------- *
 *                       OpenSim:  ControlConstant.cpp                        *
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
#include "ControlConstant.h"


//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================
//_____________________________________________________________________________


using namespace OpenSim;
/**
 * Destructor.
 */
ControlConstant::~ControlConstant()
{
}
//_____________________________________________________________________________
ControlConstant::ControlConstant(double aX,const char *aName) :
    _x(_propX.getValueDbl())
{
    setNull();
    setIsModelControl(false);
    _x = aX;
    setName(aName);
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aControl Control to copy.
 */
ControlConstant::ControlConstant(const ControlConstant &aControl) :
    Control(aControl),
    _x(_propX.getValueDbl())
{
    setNull();
    copyData(aControl);
}


//=============================================================================
// CONSTRUCTION/DESTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the member data to their NULL values.
 */
void ControlConstant::
setNull()
{
    setupProperties();
    _x = 0.0;
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void ControlConstant::
setupProperties()
{
    _propX.setName("value");
    _propX.setValue(0.0);
    _propertySet.append( &_propX );
}
//_____________________________________________________________________________
/**
 * Copy the member variables of the specified control.
 */
void ControlConstant::
copyData(const ControlConstant &aControl)
{
    _x = aControl.getParameterValue(0);
}


//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// ASSIGNMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return  Reference to the altered object.
 */
ControlConstant& ControlConstant::
operator=(const ControlConstant &aControl)
{
    // BASE CLASS
    Control::operator=(aControl);

    // DATA
    copyData(aControl);

    return(*this);
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// NUMBER OF PARAMETERS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the number of parameters that are used to specify the control curve.
 *
 * @return Number of parameters.
 */
int ControlConstant::
getNumParameters() const
{
    return(1);
}

//-----------------------------------------------------------------------------
// PARAMETER MIN
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the minimum value of a control parameter.
 *
 * @param aI Index of the parameter.
 * @param aMin Minimum value of the parameter.
 */
void ControlConstant::
setParameterMin(int aI,double aMin)
{
    setDefaultParameterMin(aMin);
}
//_____________________________________________________________________________
/**
 * Get the minimum value of a control parameter.
 *
 * @param aI Index of the parameter.
 * @return Minimum value of the parameter.
 */
double ControlConstant::
getParameterMin(int aI) const
{
    return(getDefaultParameterMin());
}

//-----------------------------------------------------------------------------
// PARAMETER MAX
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the maximum value of a control parameter.
 *
 * @param aI Index of the parameter.
 * @param aMax Maximum value of the parameter.
 */
void ControlConstant::
setParameterMax(int aI,double aMax)
{
    setDefaultParameterMax(aMax);
}
//_____________________________________________________________________________
/**
 * Get the maximum value of a control parameter.
 *
 * @param aI Index of the parameter.
 * @return Maximum value of the parameter.
 */
double ControlConstant::
getParameterMax(int aI) const
{
    return(getDefaultParameterMax());
}

//-----------------------------------------------------------------------------
// PARAMETER TIME
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
double ControlConstant::
getParameterTime(int aI) const
{
    return(SimTK::NaN);
}

//-----------------------------------------------------------------------------
// PARAMETER NEIGHBORHOOD
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
void ControlConstant::
getParameterNeighborhood(int aI,double &rTLower,double &rTUpper) const
{
    rTLower = -SimTK::Infinity;
    rTUpper =  SimTK::Infinity;
}

//-----------------------------------------------------------------------------
// PARAMETER LIST
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
int ControlConstant::
getParameterList(double aT,Array<int> &rList)
{
    rList.setSize(0);
    rList.append(0);
    return(rList.getSize());
}
//_____________________________________________________________________________
int ControlConstant::
getParameterList(double aTLower,double aTUpper,Array<int> &rList)
{
    rList.setSize(0);
    return(rList.getSize());
}

//-----------------------------------------------------------------------------
// PARAMETER VALUE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
void ControlConstant::
setParameterValue(int aI,double aX)
{
    _x = aX;
}
//_____________________________________________________________________________
double ControlConstant::
getParameterValue(int aI) const
{
    return(_x);
}

//-----------------------------------------------------------------------------
// CONTROL VALUE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
void ControlConstant::
setControlValue(double aT,double aX)
{
    _x = aX;
}
//_____________________________________________________________________________
double ControlConstant::
getControlValue(double aT)
{
    return(_x);
}

//-----------------------------------------------------------------------------
// CONTROL VALUE MIN
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
void ControlConstant::
setControlValueMin(double aT,double aMin)
{
    setDefaultParameterMin(aMin);
}
//_____________________________________________________________________________
double ControlConstant::
getControlValueMin(double aT)
{
    return( getDefaultParameterMin() );
}

//-----------------------------------------------------------------------------
// CONTROL VALUE MAX
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
void ControlConstant::
setControlValueMax(double aT,double aMax)
{
    setDefaultParameterMax(aMax);
}
//_____________________________________________________________________________
double ControlConstant::
getControlValueMax(double aT)
{
    return( getDefaultParameterMax() );
}


