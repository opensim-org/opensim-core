#ifndef OPENSIM_PROPERTY_OBJ_PTR_H_
#define OPENSIM_PROPERTY_OBJ_PTR_H_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  PropertyObjPtr.h                         *
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

// INCLUDES
#include "osimCommonDLL.h"
#include <string>
#include "Object.h"
#include "Property_Deprecated.h"

//=============================================================================
//=============================================================================
namespace OpenSim { 

/**
 * Class PropertyObjPtr extends class Property.  It differs from ProprtyObj in
 * that PropertyObj looks for  an XML element with a specific tag name
 * (matching the object type), PropertyObjPtr accepts any recognized object
 * type.  It's essentially like a singleton PropertyObjArray.
 *
 * Note: Class owns its Object _value (so it is deleted as appropriate).
 *
 * @version 1.0
 * @author Eran Guendelman
 */
template<class T=Object> class PropertyObjPtr : public Property_Deprecated
{

//=============================================================================
// DATA
//=============================================================================
private:
    /** Value. */
    T *_value;

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    PropertyObjPtr() : Property_Deprecated(ObjPtr, "ObjPtrPropertyName")
    {
        _value = 0;
        setAllowableListSize(0,1);
    }

    PropertyObjPtr(const PropertyObjPtr<T> &aProperty)
        : Property_Deprecated(aProperty)
    {
        _value = aProperty._value ? aProperty._value->clone() : 0;
    }

    PropertyObjPtr* clone() const override
    {
        return new PropertyObjPtr<T>(*this);
    }

    virtual ~PropertyObjPtr()
    { 
        delete _value; 
    }

    int getNumValues() const override {return _value?1:0;}
    bool isObjectProperty() const override {return true;}
    bool isAcceptableObjectTag
        (const std::string& objectTypeTag) const override {return true;}
    const Object& getValueAsObject(int index=-1) const override
    {   assert(index <= 0); return *_value; }
    Object& updValueAsObject(int index=-1) override
    {   assert(index <= 0); return *_value; }
    void setValueAsObject(const Object& obj, int index=-1) override
    {   assert(index <= 0); setValue(obj.clone()); }

    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
public:
    PropertyObjPtr<T>& operator=(const PropertyObjPtr &aProperty)
    {
        Property_Deprecated::operator =(aProperty);
        delete _value;
        _value = aProperty._value ? aProperty._value->clone() : 0;
        return *this;
    }

    void assign(const AbstractProperty& that) override {
        try {
            *this = dynamic_cast<const PropertyObjPtr&>(that);
        } catch(const std::bad_cast&) {
            OPENSIM_THROW(InvalidArgument,
                          "Unsupported type. Expected: " + this->getTypeName() +
                          " | Received: " + that.getTypeName());
        }
    }

    bool operator==(const Property_Deprecated& aProperty) const override {
        bool equal = Property_Deprecated::operator==(aProperty);
        if (equal){ 
            if (_value==NULL) return (((PropertyObjPtr&) aProperty)._value==NULL);
            return ((*_value) == (*((PropertyObjPtr&) aProperty)._value));
        }
        return equal;
    }
    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------
public:
    // TYPE
    std::string getTypeName() const override
    {   return T::getClassName(); }
    // VALUE
    bool isValidObject(const Object *aValue) const override { return dynamic_cast<const T*>(aValue)!=0; }
    void setValue(Object *aValue) override { delete _value; _value = dynamic_cast<T*>(aValue); }
    const Object* getValueObjPtr() const override { return _value; }
    T*& getValueObjPtrRef() { return _value; }
    // VALUE as String
    std::string toString() const override {return "(ObjectPointer)";}

//=============================================================================
};  // END of class PropertyObjPtr

}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_PROPERTY_OBJ_PTR_H_
