#ifndef __IKCoordinateTask_h__
#define __IKCoordinateTask_h__
/* -------------------------------------------------------------------------- *
 *                        OpenSim:  IKCoordinateTask.h                        *
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

#include "osimToolsDLL.h"
#include "IKTask.h"
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyDbl.h>

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * @authors Eran Guendelman
 * @version 1.0
 */

class OSIMTOOLS_API IKCoordinateTask : public IKTask {
OpenSim_DECLARE_CONCRETE_OBJECT(IKCoordinateTask, IKTask);

public:
    enum ValueType { DefaultValue, ManualValue, FromFile };

protected:
    PropertyStr _valueTypeProp;
    std::string &_valueType;

    PropertyDbl _valueProp;
    double &_value;

public:
    IKCoordinateTask();
    IKCoordinateTask(const IKCoordinateTask &aIKCoordinateTask);

#ifndef SWIG
    IKCoordinateTask& operator=(const IKCoordinateTask &aIKCoordinateTask);
#endif

    void setValueType(ValueType type) { _valueType = ValueTypeToString(type); }
    ValueType getValueType() const { return StringToValueType(_valueType); }

    double getValue() const { return _value; }
    void setValue(double value) { _value = value; }

    static std::string ValueTypeToString(ValueType type);
    static ValueType StringToValueType(const std::string &str);

private:
    void setupProperties();
//=============================================================================
};  // END of class IKCoordinateTask
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __IKCoordinateTask_h__
