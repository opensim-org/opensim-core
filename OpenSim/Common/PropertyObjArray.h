#ifndef OPENSIM_PROPERTY_OBJ_ARRAY_H_
#define OPENSIM_PROPERTY_OBJ_ARRAY_H_
/* -------------------------------------------------------------------------- *
 *                        OpenSim:  PropertyObjArray.h                        *
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
#include "ArrayPtrs.h"
#include "Property_Deprecated.h"
#include "Object.h"


//=============================================================================
//=============================================================================
namespace OpenSim { 

/**
 * Class PropertyObjArray extends class Property.
 * Assumes template T is a class derived from Object.
 *
 * @version 1.0
 * @author Frank C. Anderson
 */
template<class T = Object>
class PropertyObjArray : public Property_Deprecated
{

//=============================================================================
// DATA
//=============================================================================
private:
    /** Array of objects. */
    ArrayPtrs<T> _array;

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    PropertyObjArray(const std::string &aName = "",const ArrayPtrs<T> &aArray = ArrayPtrs<T>()) 
    :   Property_Deprecated(ObjArray, aName), _array(aArray) {}
    PropertyObjArray(const PropertyObjArray<T> &aProperty) 
    :   Property_Deprecated(aProperty) { _array = aProperty._array; }

    bool isArrayProperty() const override {return true;}

    PropertyObjArray* clone() const override
    {   return new PropertyObjArray<T>(*this); }

    int getNumValues() const override {return getArraySize();}
    bool isObjectProperty() const override {return true;}
    bool isAcceptableObjectTag
        (const std::string& objectTypeTag) const override {return true;}
    const Object& getValueAsObject(int index) const override
    {  return *getValueObjPtr(index); }
    Object& updValueAsObject(int index) override
    {
        return const_cast<Object&>(*getValueObjPtr(index));
    }
    void setValueAsObject(const Object& obj, int index) override
    {   _array.set(index, dynamic_cast<T*>(obj.clone())); }

    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
public:
    PropertyObjArray& operator=(const PropertyObjArray &aProperty) {
        Property_Deprecated::operator=(aProperty);
        _array = aProperty._array;
        return (*this);
    }

    void assign(const AbstractProperty& that) override {
        try {
            *this = dynamic_cast<const PropertyObjArray&>(that);
        } catch(const std::bad_cast&) {
            OPENSIM_THROW(InvalidArgument,
                          "Unsupported type. Expected: " + this->getTypeName() +
                          " | Received: " + that.getTypeName());
        }
    }

    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------
public:
    bool isValidObject(const Object *obj) const override
    { return dynamic_cast<const T*>(obj)!=0; }
    // TYPE
    std::string getTypeName() const override
    {   return T::getClassName(); }
    // VALUE as String
    std::string toString() const override
    {return "(Array of objects)";}
    // SIZE
    int getArraySize() const override { return _array.getSize(); }
    // VALUE
    const Object* getValueObjPtr(int index) const override { return (Object*)_array.get(index); }
    void appendValue(Object *obj) override { 
        if(!isValidObject(obj)) 
            throw Exception("PropertyObjArray: ERR- Attempting to append invalid object of type "
            + obj->getConcreteClassName(), __FILE__,__LINE__);
        _array.append(static_cast<T*>(obj));
    }
    void clearObjArray() override { _array.setSize(0); }

    // Other members (not in Property base class)
    void setValue(const ArrayPtrs<T> &aArray) { _array = aArray; }
    ArrayPtrs<T>& getValueObjArray() { return _array; }
#ifndef SWIG
    const ArrayPtrs<T>& getValueObjArray() const { return _array; }
    bool operator==(const Property_Deprecated& aProperty) const override {
        // base class
        bool equal=(Property_Deprecated::operator==(aProperty));
        if (equal) {
            PropertyObjArray& other = ((PropertyObjArray&)aProperty);
            if (_array.getSize()>0 && other._array.getSize()>0){    
                if (_array.getSize()==other._array.getSize()){
                    for(int i=0; i<_array.getSize() && equal; i++){
                        equal = (*(_array.get(i)))==(*(other._array.get(i)));
                    }
                    return equal;
                }
                else
                    return false;
            }
            else 
                return ((_array.getSize()==0) && (other._array.getSize()==0));
        }
        return equal;
    }
#endif

//=============================================================================
};  // END of class PropertyObjArray

} //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_PROPERTY_OBJ_ARRAY_H_
