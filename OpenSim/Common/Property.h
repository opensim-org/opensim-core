#ifndef _Property_h_
#define _Property_h_
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
#include <string>
#include "Array.h"
#include "ArrayPtrs.h"

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
#endif

#define Property_PROPERTY_TYPE_MISMATCH() \
	throw Exception(std::string(__FUNCTION__)+": Property type mismatch. This property is of type "+getTypeAsString()+".",__FILE__,__LINE__);

class OSIMCOMMON_API Property  
{

//=============================================================================
// DATA
//=============================================================================
public:
	/** Enumeration of recognized types. */
	enum PropertyType
	{
		None=0,Bool, Int, Dbl, Str, Obj, ObjPtr,
		BoolArray, IntArray, DblArray, StrArray, ObjArray,
		DblVec3, 
		Transform // 3 BodyFixed X,Y,Z Rotations followed by 3 Translations
		//Station	   Point on a Body: String, Vec3 
	};

	enum PropertyCategory
	{
		NoCategory,
		Display,
		System,
		Dynamics
	};

private:
	/** Type of the property. */
	PropertyType _type;
	/** Name of the property. */
	std::string _name;
	/** Category of property */
	PropertyCategory _category;

	/** Flag indicating whether or not this property uses some
	default property for initializing its value. */
	bool _useDefault;

	int _minArraySize; // minimum number of elements for a property of array type
	int _maxArraySize; // maximum number of elements for a property of array type
protected:
	/** Comment to be associated with property, shown for default objects only
	for efficiency. */
	std::string _comment;
	/** String representation of property */
	std::string _valueString;
//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	Property();
	Property(PropertyType aType,const std::string &aName);
	Property(const Property &aProperty);
	virtual Property* copy() const=0;
	virtual ~Property() {};
	void setNull();

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	Property& operator=(const Property &aProperty);
	virtual bool operator==(const Property &aProperty) const;
	virtual bool operator<(const Property &aProperty) const;
	friend std::ostream& operator<<(std::ostream &aOut,const Property &aProperty) {
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

	// NAME
	void setName(const std::string &aName);
	const std::string& getName() const;

	// Comment
	void setComment(const std::string &aComment) { _comment = aComment; };
	const std::string& getComment() const { return _comment; };
	// VALUE
	// Textual representation
	virtual const std::string &toString()=0;

	void setAllowableArraySize(int aMin, int aMax) { _minArraySize = aMin; _maxArraySize = aMax; }
	void setAllowableArraySize(int aNum) { _minArraySize = _maxArraySize = aNum; }
	int getMinArraySize() { return _minArraySize; }
	int getMaxArraySize() { return _maxArraySize; }

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
	// USE DEFAULT
	void setUseDefault(bool aTrueFalse);
	bool getUseDefault() const;

	// Generic way to get number of elements
	virtual int getArraySize() const { Property_PROPERTY_TYPE_MISMATCH(); }

	// Templates for get & set
	template<class T> T &getValue();
	template<class T> const T &getValue() const;
	template<class T> Array<T> &getValueArray();
	template<class T> const Array<T> &getValueArray() const;

//=============================================================================
};	// END of class Property

// Specializations of template get/set
// Must be inline! (Trying to put function bodies in cpp fails with an internal compiler error in VC7.1)
template<> inline bool &Property::getValue() { return getValueBool(); }
template<> inline const bool &Property::getValue() const { return getValueBool(); }
template<> inline int &Property::getValue() { return getValueInt(); }
template<> inline const int &Property::getValue() const { return getValueInt(); }
template<> inline double &Property::getValue() { return getValueDbl(); }
template<> inline const double &Property::getValue() const { return getValueDbl(); }
template<> inline std::string &Property::getValue() { return getValueStr(); }
template<> inline const std::string &Property::getValue() const { return getValueStr(); }

template<> inline Array<bool> &Property::getValueArray() { return getValueBoolArray(); }
template<> inline const Array<bool> &Property::getValueArray() const { return getValueBoolArray(); }
template<> inline Array<int> &Property::getValueArray() { return getValueIntArray(); }
template<> inline const Array<int> &Property::getValueArray() const { return getValueIntArray(); }
template<> inline Array<double> &Property::getValueArray() { return getValueDblArray(); }
template<> inline const Array<double> &Property::getValueArray() const { return getValueDblArray(); }
template<> inline Array<std::string> &Property::getValueArray() { return getValueStrArray(); }
template<> inline const Array<std::string> &Property::getValueArray() const { return getValueStrArray(); }

}; //namespace
//=============================================================================
//=============================================================================

#endif //__Property_h__
