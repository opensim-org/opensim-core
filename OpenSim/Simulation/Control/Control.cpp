/* -------------------------------------------------------------------------- *
 *                           OpenSim:  Control.cpp                            *
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
#include "Control.h"


using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
Control::~Control()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
Control::Control(const char *aName) :
    _isModelControl( _propIsModelControl.getValueBool() ),
    _extrapolate( _propExtrapolate.getValueBool() ),
    _defaultMin( _propDefaultMin.getValueDbl() ),
    _defaultMax( _propDefaultMax.getValueDbl() ),
    _filterOn( _propFilterOn.getValueBool() )
{
    setNull();
    setName(aName);
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aControl Control to copy.
 */
Control::Control(const Control &aControl) :
    Object(aControl),
    _isModelControl( _propIsModelControl.getValueBool() ),
    _extrapolate( _propExtrapolate.getValueBool() ),
    _defaultMin( _propDefaultMin.getValueDbl() ),
    _defaultMax( _propDefaultMax.getValueDbl() ),
    _filterOn( _propFilterOn.getValueBool() )
{
    setNull();
    copyData(aControl);
}


//=============================================================================
// CONSTRUCTION/DESTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the member variables to their NULL values.
 */
void Control::
setNull()
{
    //generateProperties();
    setupProperties();
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void Control::
setupProperties()
{
    _propIsModelControl.setName("is_model_control");
    _propIsModelControl.setValue(true);
    _propertySet.append(&_propIsModelControl);

    _propExtrapolate.setName("extrapolate");
    _propExtrapolate.setValue(true);
    _propertySet.append(&_propExtrapolate);

    _propDefaultMin.setName("default_min");
    _propDefaultMin.setValue(0.02);
    _propertySet.append(&_propDefaultMin);

    _propDefaultMax.setName("default_max");
    _propDefaultMax.setValue(1.0);
    _propertySet.append(&_propDefaultMax);

    _propFilterOn.setName("filter_on");
    _propFilterOn.setValue(false);
    _propertySet.append( &_propFilterOn );
}
//_____________________________________________________________________________
void Control::
copyData(const Control &aControl)
{
    _isModelControl = aControl._isModelControl;
    _extrapolate = aControl._extrapolate;
    _defaultMin = aControl._defaultMin;
    _defaultMax = aControl._defaultMax;
    _filterOn = aControl.getFilterOn();
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
Control& Control::
operator=(const Control &aControl)
{
    // BASE CLASS
    Object::operator=(aControl);

    // DATA
    copyData(aControl);

    return(*this);
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// IS MODEL CONTROL
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
void Control::
setIsModelControl(bool aTrueFalse)
{
    _isModelControl = aTrueFalse;
}
//_____________________________________________________________________________
bool Control::
getIsModelControl() const
{
    return(_isModelControl);
}

//-----------------------------------------------------------------------------
// EXTRAPOLATE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
void Control::
setExtrapolate(bool aTrueFalse)
{
    _extrapolate = aTrueFalse;
}
//_____________________________________________________________________________
bool Control::
getExtrapolate() const
{
    return(_extrapolate);
}

//-----------------------------------------------------------------------------
// DEFAULT PARAMETER MINIMUM
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
void Control::
setDefaultParameterMin(double aMin)
{
    _defaultMin = aMin;
    if(_defaultMax < _defaultMin) {
        printf("Control.setDefaultParameterMin: ");
        printf("WARN- minimum is greater than maximum, setting max = min.\n");
        _defaultMax = _defaultMin;
    }
}
//_____________________________________________________________________________
double Control::
getDefaultParameterMin() const
{
    return(_defaultMin);
}

//-----------------------------------------------------------------------------
// DEFAULT PARAMETER MAXIMUM
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
void Control::
setDefaultParameterMax(double aMax)
{
    _defaultMax = aMax;
    if(_defaultMin > _defaultMax) {
        printf("Control.setDefaultParameterMin: ");
        printf("WARN- maximum is less than minimum, setting min = max.\n");
        _defaultMin = _defaultMax;
    }
}
//_____________________________________________________________________________
double Control::
getDefaultParameterMax() const
{
    return(_defaultMax);
}

//-----------------------------------------------------------------------------
// FILTER ON
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
void Control::
setFilterOn(bool aTrueFalse)
{
    _filterOn = aTrueFalse;
}
//_____________________________________________________________________________
bool Control::
getFilterOn() const
{
    return(_filterOn);
}

// Convenience methods
//_____________________________________________________________________________
double Control::getFirstTime() const
{
    string msg = "Control.getFirstTime: This method must be overridden.";
    throw(Exception(msg,__FILE__,__LINE__));
    return 0;
}
//_____________________________________________________________________________
double Control::getLastTime() const
{
    string msg = "Control.getLastTime: This method must be overridden.";
    throw(Exception(msg,__FILE__,__LINE__));
    return 1; 
}


//-----------------------------------------------------------------------------
// SIMPLIFY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
void Control::
simplify(const PropertySet &aProperties)
{
    string msg = "Control.simplify: This method must be overridden.";
    throw(Exception(msg,__FILE__,__LINE__));
}


//-----------------------------------------------------------------------------
// FILTER
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
void Control::
filter(double aT)
{
}
