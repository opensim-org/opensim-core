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
	/** Enumeration of recognized types. */
	enum PropertyType
	{
		None=0, Bool, Int, Dbl, Str, Obj, ObjPtr,
		BoolArray, IntArray, DblArray, StrArray, ObjArray,
		DblVec, DblVec3,
		Transform // 3 BodyFixed X,Y,Z Rotations followed by 3 Translations
		//Station	   Point on a Body: String, Vec3 
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
    /** Return the current value of this property in a string suitable for
    displaying to a user in the GUI. **/
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

/** This class defines the external string representation for a property type T.
In case you don't like the name you get from typeid() (and
you probably won't -- it will vary across platforms), you should specialize
this class to provide a nicer name. When a new type is defined that can be
used as a property type, add a specialization of this class in that header.
Here we'll specialize for the built-in, std:: and SimTK:: types and some
basic OpenSim:: types. **/
template <class T> struct PropertyTypeName {
    static const char* name() {return typeid(T).name();}
};

template<> struct PropertyTypeName<bool> 
{   static const char* name() {return "bool";} };
template<> struct PropertyTypeName<int> 
{   static const char* name() {return "int";} };
template<> struct PropertyTypeName<double> 
{   static const char* name() {return "double";} };
template<> struct PropertyTypeName<std::string> 
{   static const char* name() {return "string";} };
template<> struct PropertyTypeName<SimTK::Vec3> 
{   static const char* name() {return "Vec3";} };
template<> struct PropertyTypeName<SimTK::Transform> 
{   static const char* name() {return "Transform";} };

template<> struct PropertyTypeName< Array<bool> > 
{   static const char* name() {return "Array<bool>";} };
template<> struct PropertyTypeName< Array<int> > 
{   static const char* name() {return "Array<int>";} };
template<> struct PropertyTypeName< Array<double> > 
{   static const char* name() {return "Array<double>";} };
template<> struct PropertyTypeName< Array<std::string> > 
{   static const char* name() {return "Array<string>";} };


}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_ABSTRACT_PROPERTY_H_
