#ifndef _PropertyObjArray_h_
#define _PropertyObjArray_h_
// PropertyObjArray.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c) 2005, Stanford University. All rights reserved. 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions
* are met: 
*  - Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer. 
*  - Redistributions in binary form must reproduce the above copyright 
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the distribution. 
*  - Neither the name of the Stanford University nor the names of its 
*    contributors may be used to endorse or promote products derived 
*    from this software without specific prior written permission. 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
* POSSIBILITY OF SUCH DAMAGE. 
*/

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


// INCLUDES
#include "osimCommonDLL.h"
#include <string>
#include "ArrayPtrs.h"
#include "Property.h"


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
class PropertyObjArray : public Property
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
		: Property(Property::ObjArray, aName), _array(aArray) {}
	PropertyObjArray(const PropertyObjArray<T> &aProperty) : Property(aProperty) { _array = aProperty._array; }
	virtual Property* copy() const { return new PropertyObjArray<T>(*this); }

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
	PropertyObjArray& operator=(const PropertyObjArray &aProperty) {
		Property::operator =(aProperty);
		_array = aProperty._array;
		return (*this);
	}

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
public:
	virtual bool isValidObject(const Object *obj) const { return dynamic_cast<const T*>(obj)!=0; }
	// TYPE
	virtual const char* getTypeAsString() const { return "ObjArray"; }
	// VALUE as String
	virtual const std::string &toString() { _valueString = "(Array of objects)"; return _valueString; }
	// VALUE
	virtual int getValueObjArraySize() const { return _array.getSize(); }
	virtual Object* getValueObjPtr(int index) { return (Object*)_array.get(index); }
	virtual void appendValue(Object *obj) { 
		if(!isValidObject(obj)) throw Exception("PropertyObjArray: ERR- Attempting to append invalid object of type "+obj->getType(),__FILE__,__LINE__);
		_array.append(static_cast<T*>(obj));
	}
	virtual void clearObjArray() { _array.setSize(0); }

	// Other members (not in Property base class)
	void setValue(const ArrayPtrs<T> &aArray) { _array = aArray; }
	ArrayPtrs<T>& getValueObjArray() { return _array; }
#ifndef SWIG
	const ArrayPtrs<T>& getValueObjArray() const { return _array; }
#endif

//=============================================================================
};	// END of class PropertyObjArray

} //namespace
//=============================================================================
//=============================================================================

#endif //__PropertyObjArray_h__
