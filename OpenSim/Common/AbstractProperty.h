#ifndef OPENSIM_ABSTRACT_PROPERTY_H_
#define OPENSIM_ABSTRACT_PROPERTY_H_
// AbstractProperty.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2012, Stanford University. All rights reserved. 
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
#include "SimTKsimbody.h"

#include <string>
#include <cmath>
#include <typeinfo>
#include <cassert>

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A property consists of a type, name, and a value or an array of values.
 *
 * AbstractProperty is an abstract base class that provides the functionality
 * common to all property types.
 *
 * @author Cassidy Kelly, Ajay Seth
 */

class OSIMCOMMON_API AbstractProperty
{
public:
    // TODO: this enumeration should not be necessary.
	/** Enumeration of recognized types. */
	enum PropertyType
	{
		None=0, Bool, Int, Dbl, Str, Obj, ObjPtr,
		BoolArray, IntArray, DblArray, StrArray, ObjArray,
		DblVec, DblVec3,
		Transform // 3 BodyFixed X,Y,Z Rotations followed by 3 Translations
	};

	AbstractProperty();
	AbstractProperty(const std::string& name, 
                     const std::string& typeAsString, 
                     const std::string& comment);
	
    // Default copy constructor and copy assignment operator.

    #ifndef SWIG
    /** See the equals() method for the meaning of this operator. **/
    bool operator==(const AbstractProperty& other) const
    {   return equals(other); }
    #endif

    // This is the interface that any concrete Property class must implement.
    //--------------------------------------------------------------------------
	/** Return all heap space used by this property. **/
    virtual ~AbstractProperty() {}
    /** Return a new instance of this concrete property object. This is
    allocated on the heap and it is up to the caller to delete it when done. **/
	virtual AbstractProperty* copy() const = 0;
    /** The meaning of equals() is determined by the concrete property
    depending on its type. Floating point values should be compared to a
    tolerance, and should be considered equal if both are the same infinity
    or both are NaN (the latter in contrast to normal IEEE floating point 
    behavior, where NaN!=NaN). **/
	virtual bool equals(const AbstractProperty& other) const = 0;
    /** Return the enum value corresponding to the concrete property. **/
	virtual PropertyType getPropertyType() const = 0;
    /** For relatively simple types, return the current value of this property 
    in a string suitable for displaying to a user in the GUI. Objects just
    return something like "(Object)". **/
    virtual std::string toString() const = 0;
    /** Return the current value as type T; this works only if the underlying
    concrete property stores type T otherwise throws an exception. **/
    template <class T> const T& getValue() const;
    /** Return writable access to the current value as type T; this works only
    if the underlying concrete property is actually of type T. Otherwise it
    throws an exception. **/
    template <class T> T& updValue();
    //--------------------------------------------------------------------------

    /** Set the property name. **/
	void setName(const std::string& aName){ _name = aName; }
	/** Comment to be associated with property, shown for default objects only
	for efficiency. */
	void setComment(const std::string& aComment){ _comment = aComment; }
	/** Flag indicating whether or not this property uses some
	default property for initializing its value. */
	void setUseDefault(bool aTrueFalse) { _useDefault = aTrueFalse; }
    /** TODO: what is this? */
	void setMatchName(bool aMatchName) { _matchName = aMatchName; }
    /** For an array property, require that the number of elements n in the
    array be aMin <= n <= aMax. */
	void setAllowableArraySize(int aMin, int aMax) 
    {   assert(0 <= aMin && aMin <= aMax); 
       _minArraySize = aMin; _maxArraySize = aMax; }
    /** For an array property, require that the number of elements n in the
    array be exactly n=aNum. **/
	void setAllowableArraySize(int aNum) 
    {   assert(aNum >= 1); _minArraySize = _maxArraySize = aNum; }

	const std::string& getName() const { return _name; }
	const std::string& getTypeAsString() const { return _typeAsString; }
	const std::string& getComment() const { return _comment; }
	bool getUseDefault() const { return _useDefault; }
	bool getMatchName() const { return _matchName; }
	//int getIndex() { return _index; }
    int getMinArraySize() { return _minArraySize; }
	int getMaxArraySize() { return _maxArraySize; }

    /** Provides type-specific methods used to implement generic functionality
    at the AbstractProperty level. This class must be specialized for any 
    type T that is used in a Property<T> instantiation, unless T is an 
    Object or something derived from Object. **/
    template <class T> struct TypeHelper {
        static const char* name() {return "Obj";}
        static PropertyType getPropertyType() {return Obj;}
        static bool isEqual(const T& a, const T& b) {return a==b;}
        static std::string formatForDisplay(const T&) {return "(Object)";}
    };

protected:
    /** This is for use by the concrete property types that derive from
    AbstractProperty to provide a string we can use to represent the type
    without us having to know what it is in the base class. */
    void setTypeAsString(const char* typeName) 
    {   _typeAsString = std::string(typeName); }


private:
	void setNull();

	std::string _name;
	std::string _typeAsString;
	std::string _comment;
	bool        _useDefault;
	bool        _matchName;
	int         _minArraySize; // minimum # elements for property of array type
	int         _maxArraySize; // maximum # elements for property of array type
};


template<> struct AbstractProperty::TypeHelper<bool> {
    static const char* name() {return "bool";}
    static PropertyType getPropertyType() {return Bool;}
    static bool isEqual(bool a, bool b) {return a==b;}
    OSIMCOMMON_API static std::string formatForDisplay(bool);
};
template<> struct AbstractProperty::TypeHelper<int> {
    static const char* name() {return "int";}
    static PropertyType getPropertyType() {return Int;}
    static bool isEqual(int a, int b) {return a==b;}
    OSIMCOMMON_API static std::string formatForDisplay(int);
};
template<> struct AbstractProperty::TypeHelper<std::string> {
    static const char* name() {return "string";}
    static PropertyType getPropertyType() {return Str;}
    OSIMCOMMON_API static bool isEqual(const std::string& a, 
                                       const std::string& b);
    OSIMCOMMON_API static std::string formatForDisplay(const std::string&);
};

template<> struct AbstractProperty::TypeHelper< Array<bool> > {
    static const char* name() {return "Array<bool>";}
    static PropertyType getPropertyType() {return BoolArray;}
    OSIMCOMMON_API static bool isEqual(const Array<bool>& a, 
                                       const Array<bool>& b);
    OSIMCOMMON_API static std::string formatForDisplay(const Array<bool>&);
};
template<> struct AbstractProperty::TypeHelper< Array<int> > {
    static const char* name() {return "Array<int>";} 
    static PropertyType getPropertyType() {return IntArray;}
    OSIMCOMMON_API static bool isEqual(const Array<int>& a, 
                                       const Array<int>& b);
    OSIMCOMMON_API static std::string formatForDisplay(const Array<int>&);
};
template<> struct AbstractProperty::TypeHelper< Array<std::string> > {
    static const char* name() {return "Array<string>";} 
    static PropertyType getPropertyType() {return StrArray;}
    OSIMCOMMON_API static bool isEqual(const Array<std::string>& a, 
                                       const Array<std::string>& b);
    OSIMCOMMON_API static std::string formatForDisplay(const Array<std::string>&);
};

// Floating point values' isEqual() operator returns true if all the numbers
// are equal to within a tolerance. We also say NaN==NaN, which is not standard
// IEEE floating point behavior.
template<> struct AbstractProperty::TypeHelper<double> {
    static const char* name() {return "double";}
    static PropertyType getPropertyType() {return Dbl;}
    OSIMCOMMON_API static bool isEqual(double a, double b);
    OSIMCOMMON_API static std::string formatForDisplay(double);
};
template<> struct AbstractProperty::TypeHelper<SimTK::Vec3>  {
    static const char* name() {return "Vec3";}
    static PropertyType getPropertyType() {return DblVec3;}
    OSIMCOMMON_API static bool isEqual(const SimTK::Vec3& a, 
                                       const SimTK::Vec3& b);
    OSIMCOMMON_API static std::string formatForDisplay(const SimTK::Vec3&);
};
template<> struct AbstractProperty::TypeHelper<SimTK::Vector>  {
    static const char* name() {return "Vector";}
    static PropertyType getPropertyType() {return DblVec;}
    OSIMCOMMON_API static bool isEqual(const SimTK::Vector& a, 
                                       const SimTK::Vector& b);
    OSIMCOMMON_API static std::string formatForDisplay(const SimTK::Vector&);
};
template<> struct AbstractProperty::TypeHelper<SimTK::Transform>  {
    static const char* name() {return "Transform";}
    static PropertyType getPropertyType() {return AbstractProperty::Transform;}
    OSIMCOMMON_API static bool isEqual(const SimTK::Transform& a, 
                                       const SimTK::Transform& b);
    OSIMCOMMON_API static std::string formatForDisplay(const SimTK::Transform&);
};

template<> struct AbstractProperty::TypeHelper< Array<double> >  {
    static const char* name() {return "Array<double>";}
    static PropertyType getPropertyType() {return DblArray;}
    OSIMCOMMON_API static bool isEqual(const Array<double>& a, 
                                       const Array<double>& b);
    OSIMCOMMON_API static std::string formatForDisplay(const  Array<double>&);
};


// Partial specializations for Object* derivations and ArrayPtr<Object>. These
// must be fully defined in this header file since we don't know type O yet.
template <class O> struct AbstractProperty::TypeHelper<O*> {
    static const char* name() {return "ObjPtr";} 
    static PropertyType getPropertyType() {return ObjPtr;}
    static bool isEqual(const O* a, const O* b) {return *a==*b;}
    static std::string formatForDisplay(const O*)
    {   return "(ObjectPointer)"; }
};
template<class O> struct AbstractProperty::TypeHelper< ArrayPtrs<O> > {
    static const char* name() {return "ObjArray";} 
    static PropertyType getPropertyType() {return ObjArray;}
    static bool isEqual(const ArrayPtrs<O>& a, const ArrayPtrs<O>& b) {
        if (a.getSize() != b.getSize()) return false;
        for (int i=0; i < a.getSize(); ++i)
            if (!TypeHelper<O*>::isEqual(a.get(i),b.get(i)))
                return false;
        return true;
    }
    static std::string formatForDisplay(const ArrayPtrs<O>&)
    {   return "(Array of objects)"; }  
};

}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_ABSTRACT_PROPERTY_H_
