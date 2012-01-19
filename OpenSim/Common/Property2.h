#ifndef _Property2_h_
#define _Property2_h_
// Property.h
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

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A property consists of a type, name, and a value or an array of values.
 *
 * Class Property is an abstract base class that provides the functionality
 * common to all property types.
 *
 * At the time of the first formulation of the property classes, the only
 * property types that were envisioned are for a few fundamental data types
 * and for Object:\n\n
 * \tbool\n
 * \tint\n
 * \tfloat\n
 * \tdouble\n
 * \tstring\n
 * \tObject\n
 * \tObject pointer\n
 * \tarray of bools\n
 * \tarray of ints\n
 * \tarray of floats\n
 * \tarray of doubles\n
 * \tarray of strings\n
 * \tarray of Objects\n\n
 *
 * As additional property types are needed, they may be added however.
 *
 * @todo Make default constructors for all derived classes
 * @version 1.0
 * @author Cassidy Kelly, Ajay Seth
 */

class OSIMCOMMON_API AbstractProperty
{
private:
	std::string _name;
	std::string _type;
	std::string _comment;
	bool _useDefault;
	bool _matchName;
	int _minArraySize; // minimum number of elements for a property of array type
	int _maxArraySize; // maximum number of elements for a property of array type

public:
	/** Enumeration of recognized types. */
	enum PropertyType
	{
		None=0,Bool, Int, Dbl, Str, Obj, ObjPtr,
		BoolArray, IntArray, DblArray, StrArray, ObjArray,
		DblVec,
		Transform // 3 BodyFixed X,Y,Z Rotations followed by 3 Translations
		//Station	   Point on a Body: String, Vec3 
	};

	AbstractProperty();
	AbstractProperty(const std::string &aName, const std::string &aType, const std::string &aComment);
	AbstractProperty(const AbstractProperty &aAbstractProperty);
	virtual AbstractProperty& operator=(const AbstractProperty &aAbstractProperty);
	virtual bool equals(AbstractProperty* aAbstractPropertyPtr) const = 0;
	std::string getName() const;
	void setName(std::string aName);
	std::string getType() const;
	void setType(std::string aType);
	virtual PropertyType getPropertyType() const = 0;
	std::string getComment() const;
	void setComment(std::string aComment);
	bool getUseDefault() const { return _useDefault; }
	void setUseDefault(bool aTrueFalse) { _useDefault = aTrueFalse; }
	bool getMatchName() const { return _matchName; }
	void setMatchName(bool aMatchName) { _matchName = aMatchName; }
	void setAllowableArraySize(int aMin, int aMax) { _minArraySize = aMin; _maxArraySize = aMax; }
	void setAllowableArraySize(int aNum) { _minArraySize = _maxArraySize = aNum; }
	int getMinArraySize() { return _minArraySize; }
	int getMaxArraySize() { return _maxArraySize; }
};

template <class T>
class Property2 : public AbstractProperty
{

//=============================================================================
// DATA
//=============================================================================
private:
	T _value;

public:
	Property2();
	Property2(const std::string &aName, const std::string &aType, const std::string &aComment, const T &aValue);
	Property2(const Property2 &aProperty);
	virtual Property2& operator=(const Property2 &aProperty);
	virtual bool equals(AbstractProperty* aAbstractPropertyPtr) const;
	const T& getValue() const;
	T& updateValue();
	void setValue(const T &aValue);
	virtual PropertyType getPropertyType() const;
};

template <>
inline AbstractProperty::PropertyType Property2<double>::getPropertyType() const { return Dbl; }

template <>
inline bool Property2<double>::equals(AbstractProperty *aAbstractPropertyPtr) const
{
	Property2<double> *aPropertyPtr = dynamic_cast<Property2<double> *>(aAbstractPropertyPtr);
	if (aPropertyPtr)
		return fabs(_value - aPropertyPtr->getValue()) <= 1e-7;
	return false;
}

template <>
inline AbstractProperty::PropertyType Property2<bool>::getPropertyType() const { return Bool; }

template <>
inline AbstractProperty::PropertyType Property2<int>::getPropertyType() const { return Int; }

template <>
inline AbstractProperty::PropertyType Property2<std::string>::getPropertyType() const { 	return Str; }

template <>
inline AbstractProperty::PropertyType Property2< Array<bool> >::getPropertyType() const { return BoolArray; }

template <>
inline AbstractProperty::PropertyType Property2< Array<int> >::getPropertyType() const { return IntArray; }

template <>
inline AbstractProperty::PropertyType Property2< Array<double> >::getPropertyType() const { return DblArray; }

template <>
inline AbstractProperty::PropertyType Property2< Array<std::string> >::getPropertyType() const { return StrArray; }

/**@NOTE: Specializations involving Object must be performed in Object. */

template <typename T>
Property2<T>::Property2()
{
}

template <typename T>
Property2<T>::Property2(const std::string &aName, const std::string &aType, const std::string &aComment, const T &aValue) : AbstractProperty(aName, aType, aComment)
{
	_value = aValue;
}

template <typename T>
Property2<T>::Property2(const Property2<T> &aProperty) : AbstractProperty(aProperty)
{
	_value = aProperty._value;
}

template <typename T>
Property2<T>& Property2<T>::operator=(const Property2<T> &aProperty)
{
	AbstractProperty::operator=(aProperty);
	_value = aProperty._value;
	return *this;
}

template <typename T>
bool Property2<T>::equals(AbstractProperty* aAbstractPropertyPtr) const
{
	Property2<T>* aPropertyPtr = dynamic_cast<Property2<T>*>(aAbstractPropertyPtr);
	if (aPropertyPtr)
		return _value == aPropertyPtr->getValue();
	return false;
}

template <typename T>
const T& Property2<T>::getValue() const
{
	return _value;
}

template <typename T>
T& Property2<T>::updateValue()
{
	return _value;
}

template <typename T>
void Property2<T>::setValue(const T &aValue)
{
	_value = aValue;
}

template <typename T>
AbstractProperty::PropertyType Property2<T>::getPropertyType() const
{
	return None;
}


}; //namespace
//=============================================================================
//=============================================================================

#endif //__Property2_h__
