// IKCoordinateTask.cpp
// Author: Eran Guendelman
/*
 * Copyright (c) 2007, Stanford University. All rights reserved. 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including 
 * without limitation the rights to use, copy, modify, merge, publish, 
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included 
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
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
