#ifndef OPENSIM_PROPERTY_BOOL_H_
#define OPENSIM_PROPERTY_BOOL_H_
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  PropertyBool.h                          *
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
 * Class PropertyBool extends class Property.  It consists of a boolean
 * value and the methods for accessing and modifying this value.
 *
 * @version 1.0
 * @author Frank C. Anderson
 */
class OSIMCOMMON_API PropertyBool : public Property_Deprecated
{

//=============================================================================
// DATA
//=============================================================================
private:
    /** Value. */
    bool _value;

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    PropertyBool();
    PropertyBool(const std::string &aName,bool aValue);
    PropertyBool(const PropertyBool &aProperty);
    PropertyBool* clone() const override;

    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
public:
    PropertyBool& operator=(const PropertyBool &aProperty);

    void assign(const AbstractProperty& that) override;

    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------
public:
    // TYPE
    std::string getTypeName() const override;
    // VALUE
    void setValue(bool aValue) override;
    bool& getValueBool() override;
    const bool& getValueBool() const override;
    // VALUE as String
    std::string toString() const override;
//=============================================================================
};  // END of class PropertyBool

}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_PROPERTY_BOOL_H_
