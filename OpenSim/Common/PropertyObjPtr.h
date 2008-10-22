#ifndef _PropertyObjPtr_h_
#define _PropertyObjPtr_h_
// PropertyObjPtr.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2007, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

// INCLUDES
#include "osimCommonDLL.h"
#include <string>
#include "Object.h"
#include "Property.h"


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

template<class T=Object> class PropertyObjPtr : public Property
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
	PropertyObjPtr()
		: Property(Property::ObjPtr, "ObjPtrPropertyName")
	{
		_value = 0;
	}

	PropertyObjPtr(const PropertyObjPtr<T> &aProperty)
		: Property(aProperty)
	{
		_value = aProperty._value ? (T*)aProperty._value->copy() : 0;
	}

	virtual Property* copy() const
	{
		return new PropertyObjPtr<T>(*this);
	}

	virtual ~PropertyObjPtr() 
	{ 
		delete _value; 
	}

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
	PropertyObjPtr<T>& operator=(const PropertyObjPtr &aProperty)
	{
		Property::operator =(aProperty);
		delete _value;
		_value = aProperty._value ? (T*)aProperty._value->copy() : 0;
		return *this;
	}

	virtual bool operator==(const Property &aProperty) const {
		bool equal = Property::operator ==(aProperty);
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
	virtual const char* getTypeAsString() const { return "ObjPtr"; }
	// VALUE
	virtual bool isValidObject(const Object *aValue) const { return dynamic_cast<const T*>(aValue)!=0; }
	virtual void setValue(Object *aValue) { delete _value; _value = dynamic_cast<T*>(aValue); }
	virtual Object* getValueObjPtr() { return _value; }
	T*& getValueObjPtrRef() { return _value; }
	// VALUE as String
	virtual const std::string &toString() { _valueString = "(ObjectPointer)"; return _valueString; }

//=============================================================================
};	// END of class PropertyObjPtr

}; //namespace
//=============================================================================
//=============================================================================

#endif //__PropertyObjPtr_h__
