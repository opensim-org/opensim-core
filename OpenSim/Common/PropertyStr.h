#ifndef OPENSIM_PROPERTY_STR_H_
#define OPENSIM_PROPERTY_STR_H_
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  PropertyStr.h                           *
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


// INCLUDES
#include "osimCommonDLL.h"
#include <string>
#include "Property_Deprecated.h"


//=============================================================================
//=============================================================================
namespace OpenSim { 

/**
 * Class PropertyStr extends class Property.  It consists of a string
 * value and the methods for accessing and modifying this value.
 *
 * @version 1.0
 * @author Frank C. Anderson
 */
class OSIMCOMMON_API PropertyStr : public Property_Deprecated
{
//=============================================================================
// DATA
//=============================================================================
private:
    /** Value. */
    std::string _value;

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    PropertyStr();
    PropertyStr(const std::string &aName,
        const std::string &aValue);
    PropertyStr(const PropertyStr &aProperty);
    PropertyStr* clone() const override;

    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
public:
#ifndef SWIG
    PropertyStr& operator=(const PropertyStr &aProperty);
#endif
    void assign(const AbstractProperty& that) override;

    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------
public:
    // TYPE
    std::string getTypeName() const override;
    // VALUE
    void setValue(const std::string &aValue) override;
#ifndef SWIG
    std::string& getValueStr() override;
#endif
    const std::string& getValueStr() const override;
    // VALUE as String
    std::string toString() const override;

    // Special method to reset the value
    void clearValue() { _value = getDefaultStr(); setValueIsDefault(true); }
    static const std::string& getDefaultStr();

    bool isValidFileName() { return _value!="" && _value!=getDefaultStr(); }

//=============================================================================
};  // END of class PropertyStr

}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_PROPERTY_STR_H_
