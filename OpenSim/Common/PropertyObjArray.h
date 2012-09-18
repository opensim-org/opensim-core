#ifndef _PropertyObjArray_h_
#define _PropertyObjArray_h_
/* -------------------------------------------------------------------------- *
 *                        OpenSim:  PropertyObjArray.h                        *
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


// INCLUDES
#include "osimCommonDLL.h"
#include <string>
#include "ArrayPtrs.h"
#include "Property_Deprecated.h"
#include "Object.h"


//=============================================================================
//=============================================================================
/**
 * Class PropertyObjArray extends class Property.
 * Assumes template T is a class derived from Object.
 *
 * @version 1.0
 * @author Frank C. Anderson
 */
namespace OpenSim { 

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

    bool isArrayProperty() const OVERRIDE_11 {return true;}

	PropertyObjArray* clone() const OVERRIDE_11
    {   return new PropertyObjArray<T>(*this); }

    virtual int getNumValues() const OVERRIDE_11 {return getArraySize();}
    virtual bool isObjectProperty() const OVERRIDE_11 {return true;}
    virtual bool isAcceptableObjectTag
        (const std::string& objectTypeTag) const OVERRIDE_11 {return true;}
    virtual const Object& getValueAsObject(int index) const OVERRIDE_11
    {  return *const_cast<PropertyObjArray*>(this)->getValueObjPtr(index); }
    virtual Object& updValueAsObject(int index) OVERRIDE_11
    {   return *getValueObjPtr(index); }
    virtual void setValueAsObject(const Object& obj, int index) OVERRIDE_11
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

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
public:
	virtual bool isValidObject(const Object *obj) const { return dynamic_cast<const T*>(obj)!=0; }
	// TYPE
	virtual std::string getTypeName() const OVERRIDE_11
    {   return T::getClassName(); }
	// VALUE as String
	virtual std::string toString() const {return "(Array of objects)";}
	// SIZE
	virtual int getArraySize() const { return _array.getSize(); }
	// VALUE
	virtual Object* getValueObjPtr(int index) { return (Object*)_array.get(index); }
	virtual void appendValue(Object *obj) { 
		if(!isValidObject(obj)) 
            throw Exception("PropertyObjArray: ERR- Attempting to append invalid object of type "
            + obj->getConcreteClassName(), __FILE__,__LINE__);
		_array.append(static_cast<T*>(obj));
	}
	virtual void clearObjArray() { _array.setSize(0); }

	// Other members (not in Property base class)
	void setValue(const ArrayPtrs<T> &aArray) { _array = aArray; }
	ArrayPtrs<T>& getValueObjArray() { return _array; }
#ifndef SWIG
	const ArrayPtrs<T>& getValueObjArray() const { return _array; }
	virtual bool operator==(const Property_Deprecated& aProperty) const {
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
};	// END of class PropertyObjArray

} //namespace
//=============================================================================
//=============================================================================

#endif //__PropertyObjArray_h__
