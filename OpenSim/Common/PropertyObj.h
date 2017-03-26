#ifndef OPENSIM_PROPERTY_OBJ_H_
#define OPENSIM_PROPERTY_OBJ_H_
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  PropertyObj.h                           *
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
#include "Object.h"
#include "Property_Deprecated.h"


//=============================================================================
//=============================================================================
namespace OpenSim { 

/**
 * Class PropertyObj extends class Property.  It consists of a pointer to
 * an object and the methods for accessing and modifying this object.
 *
 * @version 1.0
 * @author Frank C. Anderson
 */
class OSIMCOMMON_API PropertyObj : public Property_Deprecated
{

//=============================================================================
// DATA
//=============================================================================
private:
    /** Value. */
    Object *_value;

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    PropertyObj();
    PropertyObj(const std::string &aName,const Object &aValue);
    PropertyObj(const PropertyObj &aProperty);

    PropertyObj* clone() const override;
    virtual ~PropertyObj();

    bool isObjectProperty() const override {return true;}
    bool isAcceptableObjectTag
        (const std::string& objectTypeTag) const override {return true;}
    const Object& getValueAsObject(int index=-1) const override
    {   assert(index <= 0); return getValueObj(); }
    Object& updValueAsObject(int index=-1) override
    {   assert(index <= 0); return getValueObj(); }
    void setValueAsObject(const Object& obj, int index=-1) override
    {   assert(index <= 0); delete _value; _value=obj.clone(); }

    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
public:
    PropertyObj& operator=(const PropertyObj &aProperty);

    void assign(const AbstractProperty& that) override;

    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------
public:
    bool isValidObject(const Object *obj) const override { return true; } // TODO: make this class templated and do type checking
    // TYPE
    std::string getTypeName() const override 
    {   return "Object"; }
    // VALUE
    // Got rid of setValue(Obj) since it would be dangerous to do so given that users of
    // PropertyObj would continue to hold a reference to the (deleted) object - Eran.
    Object& getValueObj() override;
    const Object& getValueObj() const override;
    // VALUE as String
    std::string toString() const override;

//=============================================================================
};  // END of class PropertyObj

}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_PROPERTY_OBJ_H_
