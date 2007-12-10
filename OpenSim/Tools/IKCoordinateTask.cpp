// IKCoordinateTask.cpp
// Author: Eran Guendelman
/*
 * Copyright (c)  2007, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//=============================================================================
// INCLUDES
//=============================================================================
#include "IKCoordinateTask.h"
#include <OpenSim/Common/IO.h>

//=============================================================================
// NAMESPACES
//=============================================================================
using namespace std;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
IKCoordinateTask::IKCoordinateTask() :
	_valueType(_valueTypeProp.getValueStr()),
   _value(_valueProp.getValueDbl())
{
	setType("IKCoordinateTask");
	_valueType = ValueTypeToString(DefaultValue);
	_value = 0;
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 */
IKCoordinateTask::IKCoordinateTask(const IKCoordinateTask &aIKCoordinateTask) :
   IKTask(aIKCoordinateTask),
	_valueType(_valueTypeProp.getValueStr()),
   _value(_valueProp.getValueDbl())
{
	_valueType = aIKCoordinateTask._valueType;
	_value = aIKCoordinateTask._value;
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Copy method
 */
Object* IKCoordinateTask::copy() const
{
	return new IKCoordinateTask(*this);
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void IKCoordinateTask::setupProperties()
{
	_valueTypeProp.setComment("Indicates the source of the coordinate value for this task.  Possible values are"
								     " default_value (use default value of coordinate, as specified in the model file, as the fixed target value),"
								     " manual_value (use the value specified in the value property of this task as the fixed target value),"
									  " or from_file (use the coordinate values from the coordinate data specified by the coordinates_file property).");
	_valueTypeProp.setName("value_type");
	_propertySet.append(&_valueTypeProp);

	_valueProp.setComment("This value will be used as the desired (or prescribed) coordinate value if value_type is set to manual_value.");
	_valueProp.setName("value");
	_propertySet.append(&_valueProp);
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
IKCoordinateTask& IKCoordinateTask::operator=(const IKCoordinateTask &aIKCoordinateTask)
{
	IKTask::operator=(aIKCoordinateTask);
	_valueType = aIKCoordinateTask._valueType;
	_value = aIKCoordinateTask._value;
	return *this;
}

//=============================================================================
// ValueType enum utilities
//=============================================================================
string IKCoordinateTask::ValueTypeToString(ValueType type) {
	switch(type) {
		case DefaultValue: return "default_value";
		case ManualValue: return "manual_value";
		case FromFile: return "from_file";
		default: return "";
	}
}

IKCoordinateTask::ValueType IKCoordinateTask::StringToValueType(const string &str) {
	string strLower = IO::Lowercase(str);
	if(strLower=="default_value") return DefaultValue;
	else if(strLower=="manual_value") return ManualValue;
	else if(strLower=="from_file") return FromFile;
	else throw Exception("IKCoordinateTask: ERROR- Unrecognized value type '"+str+"', expecting default_value, manual_value, or from_file.",__FILE__,__LINE__);
}
