/* -------------------------------------------------------------------------- *
 *                       OpenSim:  IKCoordinateTask.cpp                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Eran Guendelman                                                 *
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
