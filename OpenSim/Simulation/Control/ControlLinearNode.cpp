/* -------------------------------------------------------------------------- *
 *                      OpenSim:  ControlLinearNode.cpp                       *
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
#include <OpenSim/Common/IO.h>
#include "ControlLinearNode.h"


//=============================================================================
// STATIC CONSTANTS
//=============================================================================


using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor. 
 *
 * @param aT Time.
 * @param aX Control value.
 * @param aMin Minimum allowed control value.
 * @param aMax Maximum allowed control value.
 *
 */
ControlLinearNode::
ControlLinearNode(double aT,double aValue) :
    _t(_propT.getValueDbl()),
    _value(_propValue.getValueDbl())
{
    setNull();
    _t = aT;
    _value = aValue;
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aControl Control to copy.
 */
ControlLinearNode::ControlLinearNode(const ControlLinearNode &aControl) :
    Object(aControl),
    _t(_propT.getValueDbl()),
    _value(_propValue.getValueDbl())
{
    setNull();
    *this = aControl;
}


ControlLinearNode::~ControlLinearNode()
{
}
//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL value for the member variables.
 */
void ControlLinearNode::
setNull()
{
    setupProperties();
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void ControlLinearNode::
setupProperties()
{
    _propT.setName("t");
    _propT.setValue(0.0);
    _propertySet.append(&_propT);

    _propValue.setName("value");
    _propValue.setValue(0.0);
    _propertySet.append(&_propValue);
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
 * @return Reference to the altered object.
 */
ControlLinearNode& ControlLinearNode::
operator=(const ControlLinearNode &aNode)
{
    _t = aNode._t;
    _value = aNode._value;
    return(*this);
}

//-----------------------------------------------------------------------------
// EQUALITY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Equality operator.
 *
 * Equality of nodes is determined by comparing the time member variables.
 * If two nodes have the same value for time within the set equality
 * tolerance, the two nodes are considered equal:\n\n
 *
 *
 * @param aNode Node with which to evaluate equality.
 * @return True if the times are equal, false otherwise.
 */
bool ControlLinearNode::
isEqual(const ControlLinearNode &aNode) const
{
    if((_t) > aNode._t) return(false);
    if((_t) < aNode._t) return(false);
    return(true);
}

//-----------------------------------------------------------------------------
// LESS THAN
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Less than operator.
 *
 * @param aNode Node with which to evaluate less than.
 * @return True if the time of this node is less than the time of aNode.
 */
bool ControlLinearNode::
isLessThan(const ControlLinearNode &aNode) const
{
    if(_t<aNode._t) return(true);
    else return(false);
}


//=============================================================================
// SET / GET
//=============================================================================
//-----------------------------------------------------------------------------
// EQUALITY TOLERANCE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the tolerance for determining equality of nodes.
 *
 * Equality of nodes is determined by comparing the time member variables.
 * If two nodes have the same value for time within the set equality
 * tolerance, the two nodes are considered equal.
 *
 * Note that the member variable _EqualityTolerance and this method are
 * static.  So, the specified equality tolerance affects all nodes.
 *
 * @param aTol Equality tolerance.  The equality tolerance must be 0.0 or
 * positive.  If a negative tolerance is sent in, the equality tolerance
 * is set to 0.0.
 * @see ==
 *
void ControlLinearNode::
SetEqualityTolerance(double aTol)
{
    _EqualityTolerance = aTol;
    if(_EqualityTolerance<0.0) _EqualityTolerance = 0.0;
}
*/
//_____________________________________________________________________________
/**
 * Get the tolerance for determining equality of nodes.
 *
 * Equality of nodes is determined by comparing _time member variables.
 * If two nodes have the same value for _time within the set equality
 * tolerance, the two nodes are considered equal.
 *
 * Note that the member variable _EqualityTolerance and this method are
 * static.  So, the specified equality tolerance affects all nodes.
 *
 * @return Equality tolerance.
 * @see ==
 *
double ControlLinearNode::
GetEqualityTolerance()
{
    return(_EqualityTolerance);
}
*/
//-----------------------------------------------------------------------------
// TIME
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the time at which this control node occurs.
 *
 * @param aTime Time at which this control node occurs.
 */
void ControlLinearNode::
setTime(double aTime)
{
    _t = aTime;
}
//_____________________________________________________________________________
/**
 * Get the time at which this control node occurs.
 *
 * @return Time at which this control node occurs.
 */
double ControlLinearNode::
getTime() const
{
    return(_t);
}

//-----------------------------------------------------------------------------
// VALUE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the value of this control node.
 *
 * @param aValue Value of this control node.
 */
void ControlLinearNode::
setValue(double aValue)
{
    _value = aValue;
    _propValue.setValueIsDefault(false);
}
//_____________________________________________________________________________
/**
 * Get the value of this control node.
 *
 * @return Value of this control node.
 */
double ControlLinearNode::
getValue() const
{
    return(_value);
}

//=============================================================================
// UTILITY
//=============================================================================
//-----------------------------------------------------------------------------
// TO STRING
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Convert the node to a string representation.
 *
 * The caller is responsible for deleting the returned string.
 *
 * @return String representation of the node.
 * @todo Alter this method so that a string value is returned.  It is too
 * complicated to have to remember to delete the returned string.
 */
char* ControlLinearNode::
toString()
{
    int size = 8*32; 
    char *string = new char[size];
    char tmp[128];
    const char *format = IO::GetDoubleOutputFormat();

    strcpy(string,"t=");
    snprintf(tmp,128,format,_t);
    strcat(string,tmp);

    strcat(string," value=");
    snprintf(tmp,128,format,_value);
    strcat(string,tmp);

    return(string);
}


