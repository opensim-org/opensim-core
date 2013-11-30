/* -------------------------------------------------------------------------- *
 *                      OpenSim:  ControlLinearNode.cpp                       *
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
#include <OpenSim/Common/IO.h>
#include "ControlLinearNode.h"


using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================
//_____________________________________________________________________________
/**
 * @param aT Time.
 * @param aX Control value.
 *
 */
ControlLinearNode::ControlLinearNode(double aT,double aValue)
{
	setNull();
    constructProperties();

	set_t(aT);
	set_value(aValue);
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL value for the member variables.
 */
void ControlLinearNode::setNull()
{
	setAuthors("Frank Anderson");
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void ControlLinearNode::constructProperties()
{
    constructProperty_t(0.0);
    constructProperty_value(0.0);
}


//=============================================================================
// OPERATORS
//=============================================================================
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
	if((get_t()) > aNode.get_t()) return(false);
	if((get_t()) < aNode.get_t()) return(false);
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
	if(get_t()<aNode.get_t()) return(true);
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
 * Equality of nodes is dertermined by comparing _time member varaibles.
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
	sprintf(tmp,format,get_t());
	strcat(string,tmp);

	strcat(string," value=");
	sprintf(tmp,format,get_value());
	strcat(string,tmp);

	return(string);
}


