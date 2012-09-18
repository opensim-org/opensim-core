/* -------------------------------------------------------------------------- *
 *                           OpenSim:  Control.cpp                            *
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Property_Deprecated.h>
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyInt.h>
#include <OpenSim/Common/PropertyDbl.h>
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
/**
 * Copy the member variables of the specified control.
 */
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
// IS MODEL CONROL
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set whether or not this control is a model control.  A model control is
 * a control that is expected by a model. Controls that are not model
 * controls may be, for example, controls that are used to set up a
 * simulation.  Such examples might include an initial state of a model
 * (e.g., joint angle, joint angular velocity, ...) or the final time of
 * a siimulation.
 *
 * @param aTrueFalse If true, the control is treated as a model control.   If
 * false, the control is not treated as a model control.
 */
void Control::
setIsModelControl(bool aTrueFalse)
{
	_isModelControl = aTrueFalse;
}
//_____________________________________________________________________________
/**
 * Get whether or not this control is a model control.  A model control is
 * a control that is expected by a model.  Controls that are not model
 * controls may be, for example, controls that are used to set up a
 * simulation.  Such examples might include an initial state of a model
 * (e.g., joint angle, joint angular velocity, ...) or the final time of
 * a siimulation.
 *
 * @return True if this control is a model control, false otherwise.
 */
bool Control::
getIsModelControl() const
{
	return(_isModelControl);
}

//-----------------------------------------------------------------------------
// EXTRAPOLATE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set whether or not to extrapolate for control curve evaluations that
 * are outide the region of confidence for a control.
 *
 * @param aTrueFalse If true, extrapolate when needed and possible to
 * determine the value of the control curve.
 */
void Control::
setExtrapolate(bool aTrueFalse)
{
	_extrapolate = aTrueFalse;
}
//_____________________________________________________________________________
/**
 * Get whether or not to extrapolate for control curve evaluations that
 * are outide the region of confidence for a control.
 *
 * @return True if this control should use extrapolation when needed and
 * possible,
 */
bool Control::
getExtrapolate() const
{
	return(_extrapolate);
}

//-----------------------------------------------------------------------------
// DEFAULT PARAMETER MINIMUM
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the default minimum value of a control parameter.
 * The default minimum is used when no minimum value is specified.
 *
 * @param aMin Minimum value.
 */
void Control::
setDefaultParameterMin(double aMin)
{
	_defaultMin = aMin;
	if(_defaultMax < _defaultMin) {
		printf("Control.setDefaultParameterMin: ");
		printf("WARN- minimum is greater than maxium, setting max = min.\n");
		_defaultMax = _defaultMin;
	}
}
//_____________________________________________________________________________
/**
 * Get the default minimum value of a control parameter.
 * The default minimum is used when no minimum value is specified.
 *
 * @return Minimum value.
 */
double Control::
getDefaultParameterMin() const
{
	return(_defaultMin);
}

//-----------------------------------------------------------------------------
// DEFAULT PARAMETER MAXIMUM
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the default maximum value of a control parameter.
 * The default maximum is used when no maximum value is specified.
 *
 * @param aMax Maximum value.
 */
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
/**
 * Get the default maximum value of a control parameter.
 * The default maximum is used when no maximum value is specified.
 *
 * @return Maximum value.
 */
double Control::
getDefaultParameterMax() const
{
	return(_defaultMax);
}

//-----------------------------------------------------------------------------
// FILTER ON
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set whether or not to apply a filter to the control values.
 *
 * @param aTrueFalse If true, will apply a filter to the control
 * values.  If false, a filter will not be used.
 */
void Control::
setFilterOn(bool aTrueFalse)
{
	_filterOn = aTrueFalse;
}
//_____________________________________________________________________________
/**
 * Get whether or not to apply a filter to the control values.
 *
 * @return True if a filter will be used.  False otherwise.
 */
bool Control::
getFilterOn() const
{
	return(_filterOn);
}

// Convenience methods
//_____________________________________________________________________________
/**
 * Get the first time where parameter is specified. Should be overriden by derived classes
 * that have a defined min time.
 *
 * @return 0.
 */
const double Control::getFirstTime() const
{
	string msg = "Control.getFirstTime: This method must be overriden.";
	throw(Exception(msg,__FILE__,__LINE__));
	return 0;
}
//_____________________________________________________________________________
/**
 * Get the last time where parameter is specified. Should be overriden by derived classes
 * that have a defined max time.
 *
 * @return 0.
 */
const double Control::getLastTime() const
{
	string msg = "Control.getLastTime: This method must be overriden.";
	throw(Exception(msg,__FILE__,__LINE__));
	return 1; 
}


//-----------------------------------------------------------------------------
// SIMPLIFY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Simplify the control (e.g., reduce the number of points in the control
 * curve) based on a set of specified properties.  Each implementation
 * is free to require whatever properties are needed to perform
 * the simplification.  Refer to the documentation in derived classes
 * to see what properties are required.
 *
 * @param aProperties Set of properties used to perform the simplify
 * operation.
 * @throw Exception This method does nothing.  It must be overriden
 * in derived classes.
 */
void Control::
simplify(const PropertySet &aProperties)
{
	string msg = "Control.simplify: This method must be overriden.";
	throw(Exception(msg,__FILE__,__LINE__));
}


//-----------------------------------------------------------------------------
// FILTER
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Filter the control curve at a particular time.
 *
 * @param aT Time at which to compute a new, filtered control value
 */
void Control::
filter(double aT)
{
}
