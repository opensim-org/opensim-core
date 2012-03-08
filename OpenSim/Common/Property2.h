#ifndef OPENSIM_PROPERTY2_H_
#define OPENSIM_PROPERTY2_H_
// Property2.h
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
#include "Array.h"
#include "ArrayPtrs.h"
#include "AbstractProperty.h"
#include "SimTKsimbody.h"

#include <string>
#include <cmath>

namespace OpenSim {


template <class T>
class Property2 : public AbstractProperty
{

//=============================================================================
// DATA
//=============================================================================
private:
	T* _valuePtr;

public:
	Property2();
	Property2(const std::string &aName, const std::string &aComment, const T &aValue);
	Property2(const Property2 &aProperty);
	Property2& operator=(const Property2 &aProperty);

    // Implement the AbstractProperty interface.
	/*virtual*/ ~Property2() { delete _valuePtr; }
	/*virtual*/ Property2* copy() const;
	/*virtual*/ bool equals(AbstractProperty* aAbstractPropertyPtr) const;
    // This is specialized for types T below.
	/*virtual*/ PropertyType getPropertyType() const 
    {   throw Exception("Property2: Use of unspecified property."); return None;}

	const T& getValue() const { return *_valuePtr; }
	T& updValue() { return *_valuePtr; }
	void setValue(const T &aValue) { *_valuePtr = aValue; }
};

// Specialize Property2::equals() for type double to allow for a tolerance.
template <>
inline bool Property2<double>::equals(AbstractProperty *aAbstractPropertyPtr) const
{
	Property2<double> *aPropertyPtr = dynamic_cast<Property2<double> *>(aAbstractPropertyPtr);
	if (aPropertyPtr) {
		if (fabs(*_valuePtr - aPropertyPtr->getValue()) > 1e-7)
			return false;
		else
			return true;
	}
	return false;
}

template <>
inline AbstractProperty::PropertyType Property2<double>::getPropertyType() const { return Dbl; }

template <>
inline AbstractProperty::PropertyType Property2<bool>::getPropertyType() const { return Bool; }

template <>
inline AbstractProperty::PropertyType Property2<int>::getPropertyType() const { return Int; }

template <>
inline AbstractProperty::PropertyType Property2<std::string>::getPropertyType() const { return Str; }

template <>
inline AbstractProperty::PropertyType Property2< Array<bool> >::getPropertyType() const { return BoolArray; }

template <>
inline AbstractProperty::PropertyType Property2< Array<int> >::getPropertyType() const { return IntArray; }

template <>
inline AbstractProperty::PropertyType Property2< Array<double> >::getPropertyType() const { return DblArray; }

template <>
inline AbstractProperty::PropertyType Property2< Array<std::string> >::getPropertyType() const { return StrArray; }

template <>
inline AbstractProperty::PropertyType Property2<SimTK::Vec3>::getPropertyType() const { return DblVec3; }

/*
template <>
inline AbstractProperty::PropertyType Property2<Object>::getPropertyType() const { return Obj; }

template <>
inline AbstractProperty::PropertyType Property2< Array<Object> >::getPropertyType() const { return ObjArray; }

template <>
inline AbstractProperty::PropertyType Property2<Object *>::getPropertyType() const { return ObjPtr; }
*/

template <typename T>
Property2<T>::Property2() : AbstractProperty()
{
    setTypeAsString(PropertyTypeName<T>::name());
	_valuePtr = new T;
}

template <typename T>
Property2<T>::Property2(const std::string &aName, const std::string &aComment, 
                        const T &aValue) 
:   AbstractProperty(aName, PropertyTypeName<T>::name(), aComment)
{
	_valuePtr = new T(aValue);
}

template <typename T>
Property2<T>::Property2(const Property2<T> &aProperty) 
:   AbstractProperty(aProperty)
{
	_valuePtr = new T(aProperty.getValue());
}

template <typename T>
Property2<T>& Property2<T>::operator=(const Property2<T> &aProperty)
{
	AbstractProperty::operator=(aProperty);
	*_valuePtr = aProperty.getValue();
	return *this;
}

template <typename T>
Property2<T>* Property2<T>::copy() const
{
	Property2<T> *prop = new Property2<T>(*this);
	return prop;
}

template <typename T>
bool Property2<T>::equals(AbstractProperty* aAbstractPropertyPtr) const
{
	Property2<T>* aPropertyPtr = dynamic_cast<Property2<T>*>(aAbstractPropertyPtr);
	if (aPropertyPtr)
		return *_valuePtr == aPropertyPtr->getValue();
	return false;
}

}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_PROPERTY2_H_
