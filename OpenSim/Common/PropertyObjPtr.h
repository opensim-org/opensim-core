#ifndef _PropertyObjPtr_h_
#define _PropertyObjPtr_h_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  PropertyObjPtr.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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
/**
 * Class PropertyObjPtr extends class Property.  It differs from ProprtyObj
 * in that PropertyObj looks for  an XML element with a specific tag name (matching
 * the object type), PropertyObjPtr accepts any recognized object type.  It's
 * essentially like a singleton PropertyObjArray.
 *
 * Note: Class owns its Object _value (so it is deleted as appropriate).
 *
 * @version 1.0
 * @author Eran Guendelman
 */
namespace OpenSim { 

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

	PropertyObjPtr* clone() const OVERRIDE_11
	{
		return new PropertyObjPtr<T>(*this);
	}

	virtual ~PropertyObjPtr()
	{ 
		delete _value; 
	}

    virtual int getNumValues() const OVERRIDE_11 {return _value?1:0;}
    virtual bool isObjectProperty() const OVERRIDE_11 {return true;}
    virtual bool isAcceptableObjectTag
        (const std::string& objectTypeTag) const OVERRIDE_11 {return true;}
    virtual const Object& getValueAsObject(int index=-1) const OVERRIDE_11
    {   assert(index <= 0); return *_value; }
    virtual Object& updValueAsObject(int index=-1) OVERRIDE_11
    {   assert(index <= 0); return *_value; }
    virtual void setValueAsObject(const Object& obj, int index=-1) OVERRIDE_11
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

	virtual bool operator==(const Property_Deprecated& aProperty) const {
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
	virtual std::string getTypeName() const OVERRIDE_11
    {   return T::getClassName(); }
	// VALUE
	virtual bool isValidObject(const Object *aValue) const { return dynamic_cast<const T*>(aValue)!=0; }
	virtual void setValue(Object *aValue) { delete _value; _value = dynamic_cast<T*>(aValue); }
	virtual Object* getValueObjPtr() { return _value; }
	T*& getValueObjPtrRef() { return _value; }
	// VALUE as String
	virtual std::string toString() const {return "(ObjectPointer)";}

//=============================================================================
};	// END of class PropertyObjPtr

}; //namespace
//=============================================================================
//=============================================================================

#endif //__PropertyObjPtr_h__
