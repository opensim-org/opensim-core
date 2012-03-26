#ifndef OPENSIM_PROPERTY_DEPRECATED_H_
#define OPENSIM_PROPERTY_DEPRECATED_H_
// Property_Deprecated.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005-12, Stanford University. All rights reserved. 
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
#include "DebugUtilities.h"

#include <string>

namespace OpenSim { 

class Object;

#ifdef SWIG
	#ifdef OSIMCOMMON_API
		#undef OSIMCOMMON_API
		#define OSIMCOMMON_API
	#endif
#endif

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
 * @author Frank C. Anderson
 */
#ifdef WIN32
#pragma warning( disable : 4290 )	// VC++ non-ANSI Exception handling
#pragma warning( disable : 4251 )	// VC2010 no-dll export of std::string

#endif

#define Property_PROPERTY_TYPE_MISMATCH() \
	throw Exception(std::string(__FUNCTION__)+": Property type mismatch. This property is of type "+getTypeAsString()+".",__FILE__,__LINE__);

class OSIMCOMMON_API Property_Deprecated : public AbstractProperty
{
//=============================================================================
// DATA
//=============================================================================
private:
	/** Type of the property. */
	PropertyType _propertyType;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	Property_Deprecated();
	Property_Deprecated(PropertyType aType,const std::string &aName);
	Property_Deprecated(const Property_Deprecated &aProperty);

    // Implement the AbstractProperty interface.
	virtual ~Property_Deprecated() {}
    bool equals(const AbstractProperty& other) const /*override*/ {
        return operator==(dynamic_cast<const Property_Deprecated&>(other));
    }
    PropertyType getPropertyType() const /*override*/ {return _propertyType;}

    // Most property types don't contain any objects. For those this method
    // can do nothing. Any property types containing objects must override.
    void setSubPropertiesUseDefault(bool shouldUseDefault) /*override*/ {}

    // Property_Deprecated does not implement AbstractProperty::copy(); that 
    // is left to concrete Property_Deprecated objects like PropertyInt.
	Property_Deprecated* copy() const /*override*/ = 0;

	void setNull();

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	Property_Deprecated& operator=(const Property_Deprecated &aProperty);
	virtual bool operator==(const Property_Deprecated &aProperty) const;
	virtual bool operator<(const Property_Deprecated &aProperty) const;
	friend std::ostream& operator<<(std::ostream &aOut,const Property_Deprecated &aProperty) {
		aOut << aProperty.getTypeAsString() << " " << aProperty.getName();
		return(aOut);
	};
#endif
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
public:

	// TYPE
	void setType(PropertyType aType);
	PropertyType getType() const;
	virtual const char* getTypeAsString() const;

	// VALUE
	// Textual representation
	virtual std::string toString() const=0;

	// These methods have been given default implementations, rather than being made pure virtual
	// so that all classes derived from Property will not have to implement each method.
	// Bool
	virtual void setValue(bool aValue) { Property_PROPERTY_TYPE_MISMATCH(); }
#ifndef SWIG
	virtual bool& getValueBool() { Property_PROPERTY_TYPE_MISMATCH(); }
#endif
	virtual const bool& getValueBool() const { Property_PROPERTY_TYPE_MISMATCH(); }
	// Int
	virtual void setValue(int aValue) { Property_PROPERTY_TYPE_MISMATCH(); }
#ifndef SWIG
	virtual int& getValueInt() { Property_PROPERTY_TYPE_MISMATCH(); }
#endif
	virtual const int& getValueInt() const { Property_PROPERTY_TYPE_MISMATCH(); }
	// Dbl
	virtual void setValue(double aValue) { Property_PROPERTY_TYPE_MISMATCH(); }
#ifndef SWIG
	virtual double& getValueDbl() { Property_PROPERTY_TYPE_MISMATCH(); }
#endif
	virtual const double& getValueDbl() const { Property_PROPERTY_TYPE_MISMATCH(); }
	// Str
	virtual void setValue(const std::string &aValue) { Property_PROPERTY_TYPE_MISMATCH(); }
#ifndef SWIG
	virtual std::string& getValueStr() { Property_PROPERTY_TYPE_MISMATCH(); }
#endif
	virtual const std::string& getValueStr() const { Property_PROPERTY_TYPE_MISMATCH(); }
	// Bool Array
	virtual void setValue(int aSize,const bool aArray[]) { Property_PROPERTY_TYPE_MISMATCH(); }
	virtual void setValue(const Array<bool> &aArray) { Property_PROPERTY_TYPE_MISMATCH(); }
	virtual Array<bool>& getValueBoolArray() { Property_PROPERTY_TYPE_MISMATCH(); }
#ifndef SWIG
	virtual const Array<bool>& getValueBoolArray() const { Property_PROPERTY_TYPE_MISMATCH(); }
#endif
	// Int Array
	virtual void setValue(int aSize,const int aArray[]) { Property_PROPERTY_TYPE_MISMATCH(); }
	virtual void setValue(const Array<int> &aArray) { Property_PROPERTY_TYPE_MISMATCH(); }
	virtual Array<int>& getValueIntArray() { Property_PROPERTY_TYPE_MISMATCH(); }
#ifndef SWIG
	virtual const Array<int>& getValueIntArray() const { Property_PROPERTY_TYPE_MISMATCH(); }
#endif
	// Dbl Array
	virtual void setValue(int aSize,const double aArray[]) { Property_PROPERTY_TYPE_MISMATCH(); }
	virtual void setValue(const Array<double> &aArray) { Property_PROPERTY_TYPE_MISMATCH(); }
	virtual Array<double>& getValueDblArray() { Property_PROPERTY_TYPE_MISMATCH(); }
#ifndef SWIG
	virtual const Array<double>& getValueDblArray() const { Property_PROPERTY_TYPE_MISMATCH(); }
#endif
	// Str Array
	virtual void setValue(int aSize,const std::string aArray[]) { Property_PROPERTY_TYPE_MISMATCH(); }
	virtual void setValue(const Array<std::string> &aArray) { Property_PROPERTY_TYPE_MISMATCH(); }
	virtual Array<std::string>& getValueStrArray() { Property_PROPERTY_TYPE_MISMATCH(); }
#ifndef SWIG
	virtual const Array<std::string>& getValueStrArray() const { Property_PROPERTY_TYPE_MISMATCH(); }
#endif

	//--------------------------------------------------------------------------
	// Obj, ObjPtr, and ObjArray require more careful treatment
	//--------------------------------------------------------------------------
	virtual bool isValidObject(const Object *aValue) const { Property_PROPERTY_TYPE_MISMATCH(); }
	// Obj
	// Got rid of setValue(Obj) since it would be dangerous to do so given that users of
	// PropertyObj would continue to hold a reference to the (deleted) object - Eran.
	virtual Object& getValueObj() { Property_PROPERTY_TYPE_MISMATCH(); }
#ifndef SWIG
	virtual const Object& getValueObj() const { Property_PROPERTY_TYPE_MISMATCH(); }
#endif
	// ObjPtr
	virtual void setValue(Object *aValue) { Property_PROPERTY_TYPE_MISMATCH(); }
	virtual Object* getValueObjPtr() { Property_PROPERTY_TYPE_MISMATCH(); }

	// Obj Array
	virtual Object* getValueObjPtr(int index) { Property_PROPERTY_TYPE_MISMATCH(); }
	virtual void appendValue(Object *obj) { Property_PROPERTY_TYPE_MISMATCH(); }
	virtual void clearObjArray() { Property_PROPERTY_TYPE_MISMATCH(); }

	// Generic way to get number of elements
	virtual int getArraySize() const { Property_PROPERTY_TYPE_MISMATCH(); }

	// Templates for get & set
	template<class T> T &getValue();
	template<class T> const T &getValue() const;
	template<class T> Array<T> &getValueArray();
	template<class T> const Array<T> &getValueArray() const;

//=============================================================================
};	// END of class Property_Deprecated

// Specializations of template get/set
// Must be inline! (Trying to put function bodies in cpp fails with an internal compiler error in VC7.1)
template<> inline bool& Property_Deprecated::getValue() { return getValueBool(); }
template<> inline const bool& Property_Deprecated::getValue() const { return getValueBool(); }
template<> inline int& Property_Deprecated::getValue() { return getValueInt(); }
template<> inline const int& Property_Deprecated::getValue() const { return getValueInt(); }
template<> inline double& Property_Deprecated::getValue() { return getValueDbl(); }
template<> inline const double& Property_Deprecated::getValue() const { return getValueDbl(); }
template<> inline std::string& Property_Deprecated::getValue() { return getValueStr(); }
template<> inline const std::string& Property_Deprecated::getValue() const { return getValueStr(); }

template<> inline Array<bool>& Property_Deprecated::getValue() { return getValueBoolArray(); }
template<> inline const Array<bool>& Property_Deprecated::getValue() const { return getValueBoolArray(); }
template<> inline Array<int>& Property_Deprecated::getValue() { return getValueIntArray(); }
template<> inline const Array<int>& Property_Deprecated::getValue() const { return getValueIntArray(); }
template<> inline Array<double>& Property_Deprecated::getValue() { return getValueDblArray(); }
template<> inline const Array<double>& Property_Deprecated::getValue() const { return getValueDblArray(); }
template<> inline Array<std::string>& Property_Deprecated::getValue() { return getValueStrArray(); }
template<> inline const Array<std::string>& Property_Deprecated::getValue() const { return getValueStrArray(); }

template<> inline Array<bool>& Property_Deprecated::getValueArray() { return getValueBoolArray(); }
template<> inline const Array<bool>& Property_Deprecated::getValueArray() const { return getValueBoolArray(); }
template<> inline Array<int>& Property_Deprecated::getValueArray() { return getValueIntArray(); }
template<> inline const Array<int>& Property_Deprecated::getValueArray() const { return getValueIntArray(); }
template<> inline Array<double>& Property_Deprecated::getValueArray() { return getValueDblArray(); }
template<> inline const Array<double>& Property_Deprecated::getValueArray() const { return getValueDblArray(); }
template<> inline Array<std::string>& Property_Deprecated::getValueArray() { return getValueStrArray(); }
template<> inline const Array<std::string>& Property_Deprecated::getValueArray() const { return getValueStrArray(); }

}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_PROPERTY_DEPRECATED_H_
