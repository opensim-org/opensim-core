// ControlLinearNode.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c) 2005, Stanford University. All rights reserved. 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions
* are met: 
*  - Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer. 
*  - Redistributions in binary form must reproduce the above copyright 
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the distribution. 
*  - Neither the name of the Stanford University nor the names of its 
*    contributors may be used to endorse or promote products derived 
*    from this software without specific prior written permission. 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
* POSSIBILITY OF SUCH DAMAGE. 
*/

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */
#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Tools/rdTools.h>
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/IO.h>
#include <OpenSim/Tools/PropertyDbl.h>
#include "ControlLinearNode.h"


//=============================================================================
// STATIC CONSTANTS
//=============================================================================


using namespace OpenSim;
double ControlLinearNode::_EqualityTolerance = rdMath::ZERO;

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
ControlLinearNode(double aT,double aX,double aMin,double aMax) :
	_t(_propT.getValueDbl()),
	_x(_propX.getValueDbl()),
	_min(_propMin.getValueDbl()),
	_max(_propMax.getValueDbl())
{
	setNull();
	_t = aT;
	_x = aX;
	_min = aMin;
	_max = aMax;
}
//_____________________________________________________________________________
/**
 * Construct a control node from an XML Element.
 *
 * @param aElement XML element.
 */
ControlLinearNode::ControlLinearNode(DOMElement *aElement) :
	Object(aElement),
	_t(_propT.getValueDbl()),
	_x(_propX.getValueDbl()),
	_min(_propMin.getValueDbl()),
	_max(_propMax.getValueDbl())
{
	setNull();
	updateFromXMLNode();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aControl Control to copy.
 */
ControlLinearNode::ControlLinearNode(const ControlLinearNode &aControl) :
	_t(_propT.getValueDbl()),
	_x(_propX.getValueDbl()),
	_min(_propMin.getValueDbl()),
	_max(_propMax.getValueDbl())
{
	setNull();
	*this = aControl;
}
//_____________________________________________________________________________
/**
 * Construct and return a copy of this object.
 *
 * The object is allocated using the new operator, so the caller is
 * responsible for deleting the returned object.
 *
 * @return Copy of this object.
 */
Object* ControlLinearNode::
copy() const
{
	ControlLinearNode *object = new ControlLinearNode(*this);
	return(object);
}
//_____________________________________________________________________________
/**
 * Copy this control and modify the copy so that it is consistent
 * with a specified XML element node.
 *
 * The copy is constructed by first using the contructor for the DOMElement
 * in order to establish the relationship of the control with the
 * XML node.  Then, the assignment operator is used to set all member variables
 * of the copy to the values of this object.  Finally, the data members of
 * the copy are updated from the DOMElment using updateFromXMLNode().
 *
 * @param aElement XML element. 
 * @return Pointer to a copy of this actuator.
 */
Object* ControlLinearNode::
copy(DOMElement *aElement) const
{
	// ESTABLISH RELATIONSHIP WITH XML NODE
	ControlLinearNode *node = (ControlLinearNode *)this->copy();

	node->setXMLNode(aElement);

	// UPDATE BASED ON NODE
	node->updateFromXMLNode();

	return(node);
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
	setType("ControlLinearNode");
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

	_propX.setName("x");
	_propX.setValue(0.0);
	_propertySet.append(&_propX);

	_propMin.setName("min");
	_propMin.setValue(0.0);
	_propertySet.append(&_propMin);

	_propMax.setName("max");
	_propMax.setValue(1.0);
	_propertySet.append(&_propMax);
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
	_x = aNode._x;
	_min = aNode._min;
	_max = aNode._max;
	return(*this);
}

//-----------------------------------------------------------------------------
// EQUALITY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Equality operator.
 *
 * Equality of nodes is dertermined by comparing the time member varaibles.
 * If two nodes have the same value for time within the set equality
 * tolerance, the two nodes are considered equal:\n\n
 *
 *
 * @param aNode Node with which to evaluate equality.
 * @return True if the times are equal, false otherwise.
 */
bool ControlLinearNode::
operator==(const ControlLinearNode &aNode) const
{
	if((_t-_EqualityTolerance) > aNode._t) return(false);
	if((_t+_EqualityTolerance) < aNode._t) return(false);
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
operator<(const ControlLinearNode &aNode) const
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
 * Equality of nodes is dertermined by comparing the time member varaibles.
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
 */
void ControlLinearNode::
SetEqualityTolerance(double aTol)
{
	_EqualityTolerance = aTol;
	if(_EqualityTolerance<0.0) _EqualityTolerance = 0.0;
}
//_____________________________________________________________________________
/**
 * Get the tolerance for determining equality of nodes.
 *
 * Equality of nodes is dertermined by comparing _time member varaibles.
 * If two nodes have the same value for _time within the set equality
 * tolerance, the two nodes are considered equal.
 *
 * Note that the member variable _EqualityTolerance and this method are
 * static.  So, the specified equality tolerance affects all nodes.
 *
 * @return Equality tolerance.
 * @see ==
 */
double ControlLinearNode::
GetEqualityTolerance()
{
	return(_EqualityTolerance);
}

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
	_x = aValue;
	_propX.setUseDefault(false);
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
	return(_x);
}

//-----------------------------------------------------------------------------
// MIN
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the minimum allowed value of this control node.
 *
 * @param aMin Minimum allowed value of this control node.  If aMin is
 * greater than the current maximum allowed value, the current
 * maximum allowed value is set to aMin.
 */
void ControlLinearNode::
setMin(double aMin)
{
	_min = aMin;
	_propMin.setUseDefault(false);
	if(_min>_max){
		_max = _min;
		_propMax.setUseDefault(false);
	}
}
//_____________________________________________________________________________
/**
 * Get the minimum allowed value of this control node.
 *
 * @return Minimum allowed value of this control node.
 */
double ControlLinearNode::
getMin() const
{
	return(_min);
}

//-----------------------------------------------------------------------------
// MAX
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the maximum allowed value of this control node.
 *
 * @param aMax Maximum allowed value of this control node.  If aMax is
 * less than the current minimum allowed value, the current
 * minimum allowed value is set to aMax.
 */
void ControlLinearNode::
setMax(double aMax)
{
	_max = aMax;
	_propMax.setUseDefault(false);
	if(_max<_min){
		_min = _max;
		_propMin.setUseDefault(false);
	}
}
//_____________________________________________________________________________
/**
 * Get the maximum allowed value of this control node.
 *
 * @return Maximum allowed value of this control node.
 */
double ControlLinearNode::
getMax() const
{
	return(_max);
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
	sprintf(tmp,format,_t);
	strcat(string,tmp);

	strcat(string," x=");
	sprintf(tmp,format,_x);
	strcat(string,tmp);

	strcat(string," min=");
	sprintf(tmp,format,_min);
	strcat(string,tmp);

	strcat(string," max=");
	sprintf(tmp,format,_max);
	strcat(string,tmp);

	return(string);
}


