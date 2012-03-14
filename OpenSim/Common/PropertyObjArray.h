#ifndef _PropertyObjArray_h_
#define _PropertyObjArray_h_
// PropertyObjArray.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
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
	/*virtual*/ PropertyObjArray* copy() const { return new PropertyObjArray<T>(*this); }

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
	virtual const char* getTypeAsString() const { return "ObjArray"; }
	// VALUE as String
	virtual std::string toString() const {return "(Array of objects)";}
	// SIZE
	virtual int getArraySize() const { return _array.getSize(); }
	// VALUE
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
