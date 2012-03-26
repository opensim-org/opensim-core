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

    /** Delete the stored value object if necessary. **/
	~Property2() { delete _valuePtr; }
    /** Clone this object, including a new copy of the stored value object. **/
	Property2* copy() const                             FINAL_11;
    /** Determine if this property is the same as another, using a
    type-dependent criterion. **/
	bool equals(const AbstractProperty& other) const    FINAL_11;
    /** Set the "use default" flag on all the subproperties, if there are
    any for this type of property. **/
    void setSubPropertiesUseDefault(bool shouldUseDefault) FINAL_11
    {   TypeHelper<T>::setSubPropertiesUseDefault(shouldUseDefault, 
                                                  *_valuePtr); }
    /** Return the AbstractProperty::PropertyType enum assigned to type T. **/
	PropertyType getPropertyType() const                FINAL_11
    {   return TypeHelper<T>::getPropertyType(); }
    /** Format this property's value as a string suitable for display to a
    user in the GUI. This is not necessarily the right format for 
    serialization to XML. **/
    std::string toString() const                        FINAL_11
    {   return TypeHelper<T>::formatForDisplay(getValue());}

    /** Obtain a const reference to the value object stored in this 
    property. **/
	const T& getValue() const { return *_valuePtr; }
    /** Obtain a writable reference to the value object stored in this 
    property. **/
	T& updValue() { return *_valuePtr; }
    /** Set the value of this property to the supplied object. **/
	void setValue(const T& v) { *_valuePtr = v; }
};


template <typename T>
Property2<T>::Property2() : AbstractProperty()
{
    setTypeAsString(TypeHelper<T>::name());
	_valuePtr = new T;
}

template <typename T>
Property2<T>::Property2(const std::string &aName, const std::string &aComment, 
                        const T &aValue) 
:   AbstractProperty(aName, TypeHelper<T>::name(), aComment)
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
bool Property2<T>::equals(const AbstractProperty& other) const
{
	const Property2<T>* p = dynamic_cast<const Property2<T>*>(&other);
    if (p == NULL)
        return false; // Type mismatch.
    return TypeHelper<T>::isEqual(getValue(), p->getValue());
}

}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_PROPERTY2_H_
