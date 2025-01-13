#ifndef OPENSIM_ABSTRACT_PROPERTY_H_
#define OPENSIM_ABSTRACT_PROPERTY_H_
/* -------------------------------------------------------------------------- *
 *                        OpenSim:  AbstractProperty.h                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson, Cassidy Kelly, Ajay Seth, Michael A. Sherman *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

// INCLUDES
#include "Assertion.h"
#include <string>
#include <typeinfo>
#include "osimCommonDLL.h"
#include "Exception.h"
#include "SimTKcommon/internal/Xml.h"


namespace OpenSim {

class Object;
class Component;
template <class T> class Property;

//==============================================================================
/// Property Exceptions
//==============================================================================
class InvalidPropertyValue : public Exception {
public:
    InvalidPropertyValue(
        const std::string& file,
        size_t line,
        const std::string& func,
        const Object& obj,
        const std::string& propertyName,
        const std::string& errorMsg);

    InvalidPropertyValue(
        const std::string& file,
        size_t line,
        const std::string& func,
        const Component& component,
        const std::string& propertyName,
        const std::string& errorMsg);
};


//==============================================================================
//                            ABSTRACT PROPERTY
//==============================================================================
/** An abstract property is a serializable (name,value) pair, for which we
do not know the type of the value. Values may be simple types like int or 
string, or may be serializable objects derived from the %OpenSim Object class.

%AbstractProperty is an abstract base class that provides the functionality
common to all properties that does not involve knowledge of the value
type. Property\<T> derives from %AbstractProperty to represent properties 
where the type is known.

@see Property, Object

@author Cassidy Kelly, Ajay Seth, Michael Sherman
**/
class OSIMCOMMON_API AbstractProperty {
public:
    // Constructors are protected.

    /** Require that the number of values n in the value list of this property
    be in the range aMin <= n <= aMax. */
    void setAllowableListSize(int aMin, int aMax) 
    {   OPENSIM_ASSERT(0 <= aMin && aMin <= aMax);
       _minListSize = aMin; _maxListSize = aMax; }

    /** Require that the number of values n in the value list of this property
    be exactly n=aNum values. **/
    void setAllowableListSize(int aNum) 
    {   OPENSIM_ASSERT(aNum >= 1); _minListSize = _maxListSize = aNum; }
    
    // Default copy constructor and copy assignment operator.

    /** Return all heap space used by this property. **/
    virtual ~AbstractProperty() {}

    /** Return a new instance of this concrete property object, containing
    new copies of this property's values. The new property object is
    allocated on the heap and it is up to the caller to delete it when done. **/
    virtual AbstractProperty* clone() const = 0;

    /** For relatively simple types, return the current value of this property
    in a string suitable for displaying to a user in the GUI (i.e., this number
    may be rounded and not an exact representation of the actual value being
    used). Objects just return something like "(Object)".
    For `Property`s, This function calls `toStringForDisplay()` with 
    `precision = 6`.**/
    virtual std::string toString() const = 0;

    /** For relatively simple types, return the current value of this property
    in a string suitable for displaying to a user in the GUI (i.e., this number
    may be rounded and not an exact representation of the actual value being 
    used). Objects just return something like "(Object)". This differs from 
    `toString()` as it has an argument, `precision`, for controlling the number
    of digits printed to string for floats. If this function is not overridden
    in a derived class, this function uses `toString()` and the `precision` 
    argument is ignored. 
    For `Property`s, in general, this means that floats will
    be represented with the number of significant digits denoted by the
    `precision` argument, and the default formatting of `stringstream`
    determines whether or not exponential notation is used. **/
    virtual std::string toStringForDisplay(const int precision) const
    {   return toString(); }

    /** This returns a string representation of this property's value type 
    which will be the same as T::getClassName() for Object-derived types T, and
    some reasonably nice name for simple types, including at least "bool",
    "int", "double", "string", "Vec3", "Vector", and "Transform". **/
    virtual std::string getTypeName() const = 0;

    /** Return true if this is an "object property", meaning that its values
    are all concrete objects of types that ultimately derive from the %OpenSim
    serializable base class Object. If this returns true then it is safe to 
    call getValueAsObject(). Otherwise this property contains only simple types
    like "int" or "std::string", and you'll need to know the actual type in
    order to access the values. **/
    virtual bool isObjectProperty() const = 0;

    /** An unnamed property is a one-object property whose name was given as
    null or as the contained object's type tag. In that case getName() will
    return the object type tag, and the XML representation will just be the
    object, with name attribute ignored if there is one. **/
    virtual bool isUnnamedProperty() const = 0;

    /** Compare this property with another one; this is primarily used 
    for testing. The properties must be of the identical concrete type, and
    their names and other base class attributes must
    be identical (including the comment). If they both have the "use default"
    flag set then we consider the values identical without looking. Otherwise,
    we delegate to the concrete property to determine if the values are equal; 
    the meaning is determined by the concrete property depending on its type. 
    Floating point values should be compared to a tolerance, and should be 
    considered equal if both are the same infinity or both are NaN (the latter
    in contrast to normal IEEE floating point behavior, where NaN!=NaN). **/
    bool equals(const AbstractProperty& other) const
    {   if (!isSamePropertyClass(other)) return false;       
        if (getName() != other.getName()) return false;
        if (getComment() != other.getComment()) return false;
        if (getMinListSize() != other.getMinListSize()) return false;
        if (getMaxListSize() != other.getMaxListSize()) return false;

        if (size() != other.size()) return false;
        // Note: we're delegating comparison of the "use default" flags
        // because only the new Property system copies them correctly, so
        // only it should compare them.
        return isEqualTo(other); // delegate to concrete property
    }

    /** Return true if the \a other property is an object of exactly the same 
    concrete class as this one. **/
    bool isSamePropertyClass(const AbstractProperty& other) const
    {   return typeid(*this) == typeid(other); }


    #ifndef SWIG
    /** See the equals() method for the meaning of this operator. **/
    bool operator==(const AbstractProperty& other) const
    {   return equals(other); }
    #endif

    /**@name                 Container interface
    A property can be viewed as a random-access container of values. These
    methods provide a subset of the usual container methods modeled after
    std::vector. Note that any methods involving the actual property value type
    T must be templatized; they will be delegated to the concrete Property\<T>
    for resolution. **/
    /**@{**/
    /** Return the number of values currently in this property's value list. **/
    int size() const {return getNumValues();}
    /** Return true if this property's value list is currently empty. **/
    bool empty() const {return size()==0;}
    /** Empty the value list for this property; fails if zero is not an 
    allowable size for this property. **/
    void clear();

    /** For an object property, the values can be obtained as references to
    the abstract base class Object from which all the objects derive. If the
    property can hold a list of values you must provide an index to select
    the value, otherwise it is optional but if supplied must be 0. This will
    throw an exception if this is not an object property, that is, if it is
    a simple property, because its values can't be represented as an Object in
    that case.

    @param[in] index    If supplied must be 0 <= index < getNumValues().
    @returns const reference to the value as an Object 
    @see updValueAsObject(), getValue\<T>() **/
    virtual const Object& getValueAsObject(int index=-1) const = 0;
    /** Get writable access to an existing object value. Note that you can't 
    use this to install a different concrete object; see setValueAsObject()
    if you want to do that. 
    @param[in] index    If supplied must be 0 <= index < getNumValues().
    @returns writable reference to the value as an Object
    @see getValueAsObject(), updValue\<T>() **/
    virtual Object& updValueAsObject(int index=-1) = 0;
    /** %Set the indicated value element to a new copy of the supplied object.
    If you already have a heap-allocated object you're willing to give up and
    want to avoid the extra copy, use adoptValueObject(). **/
    virtual void setValueAsObject(const Object& obj, int index=-1) = 0;
    // Implementation of these non-virtual templatized methods must be 
    // deferred until the concrete property declarations are known. 
    // See Object.h.

    /** Return one of the values in this property as type T; this works only 
    if the underlying concrete property stores type T and if the indexed 
    element is present, otherwise throws an exception. **/
    template <class T> const T& getValue(int index=-1) const;
    /** Return a writable reference to one of the values in this property as 
    type T; this works only if the underlying concrete property is actually of 
    type T and the indexed element is present. Otherwise it throws an exception. **/
    template <class T> T& updValue(int index=-1);
    /** Append a new value of type T to the end of the list of values currently
    contained in this property. This works only if the underlying concrete
    property is of type T, the property holds a variable-length list, and the
    list isn't already of maximum size. 
    @returns The index assigned to this value in the list. **/
    template <class T> int appendValue(const T& value);

    /** Assign (copy) property *that* to this object. */
    virtual void assign(const AbstractProperty& that) = 0;
    /**@}**/

    /** This method sets the "use default" flag for this property and the 
    properties of any objects it contains to the given value. **/
    void setAllPropertiesUseDefault(bool shouldUseDefault);

    /** Given an XML parent element expected to contain a value for this
    property as an immediate child element, find that property element and set
    the property value from it. If no such property element can be found, the
    "use default value" attribute of this property will be set on return; that
    is not an error. However, if the property element is found but is 
    malformed or unsuitable in some way, an exception will be thrown with
    a message explaining what is wrong. **/
    void readFromXMLParentElement(SimTK::Xml::Element& parent,
                                  int                  versionNumber);

    /** Given an XML parent element, append a single child element representing
    the serialized form of this property. **/
    void writeToXMLParentElement(SimTK::Xml::Element& parent) const;


    /** %Set the property name. **/
    void setName(const std::string& name){ _name = name; }

    /** %Set a user-friendly comment to be associated with property. This will
    be displayed in XML and in "help" output for %OpenSim Objects. **/
    void setComment(const std::string& aComment){ _comment = aComment; }

    /** %Set flag indicating whether the value of this property was simply
    taken from a default object and thus should not be written out when
    serializing. **/
    void setValueIsDefault(bool isDefault) { _valueIsDefault = isDefault; }

    /** Get the property name. **/
    const std::string& getName() const { return _name; }
    /** Get the comment associated with this property. **/
    const std::string& getComment() const { return _comment; }
    /** Get the flag indicating whether the current value is just the default
    value for this property (in which case it doesn't need to be written
    out). **/
    bool getValueIsDefault() const { return _valueIsDefault; }

    /** Get the minimum number of values allowed in this property's value
    list. Will be zero for optional properties, zero for list properties 
    (unless explicitly changed), and one for one-value properties. **/
    int getMinListSize() const { return _minListSize; }
    /** Get the maximum number of values allowed in this property's value
    list. Will be unlimited for list properties (unless explicitly changed), 
    and one for optional and one-value properties. **/
    int getMaxListSize() const { return _maxListSize; }

    /** This is an "optional" property if its value list can contain at most
    one value. This is the kind of property created by the
    Object::addOptionalProperty\<T> method, for any type T. **/
    bool isOptionalProperty() const
    {   return getMinListSize()==0 && getMaxListSize()==1; }

    /** This is a "list" property if its value list can contain more than
    one value. This is the kind of property created by the
    Object::addListProperty\<T> method, for any type T. **/
    bool isListProperty() const
    {   return getMaxListSize()>1; }

    /** This is a "one-value" property if its value list must always contain
    exactly one value. This is the kind of property created by the
    Object::addProperty\<T> method, for any type T. **/
    bool isOneValueProperty() const
    {   return getMinListSize()==1 && getMaxListSize()==1; }

    /** This is a "one-object" property if it is a "one-value" property and
    it contains an Object-derived value. This is the kind of property created
    by the Object::addProperty\<T> method when T is a type derived from
    %OpenSim's Object serializable base class. One-object properties have
    a special, compact representation in XML. **/
    bool isOneObjectProperty() const 
    {   return isOneValueProperty() && isObjectProperty(); }


protected:
    AbstractProperty();
    AbstractProperty(const std::string& name, 
                     const std::string& comment);
    AbstractProperty(const AbstractProperty&)            = default;
    AbstractProperty(AbstractProperty&&)                 = default;
    AbstractProperty& operator=(const AbstractProperty&) = default;
    AbstractProperty& operator=(AbstractProperty&&)      = default;

    // This is the remainder of the interface that a concrete Property class
    // must implement, hidden from AbstractProperty users.
    //--------------------------------------------------------------------------
    /** The base class equals() method will have already done a lot of checking
    prior to calling this method, including verifying that both values are 
    non-default and that the value lists are the same size; the concrete 
    property need only compare the values.**/
    virtual bool isEqualTo(const AbstractProperty& other) const = 0;

    /** Read in a new value for this property from the XML element 
    \a propertyElement. The element is expected to have the form
    @code
        <propertyName> value(s) </propertyName>
    @endcode
    where the values may be simple (like int or double) or may be objects
    contained in child elements for which the object type name serves as
    the element tag. Note that although the XML file may contain an abbreviated
    representation for one-object properties, it will have been canonicalized
    into the above form for the purpose of reading, so concrete properties may
    assume the above form always.
    
    The format for the property value (and any of its contained objects) is 
    assumed to be the one that was in use when the given ".osim" file version
    number was current; if necessary the in-memory version will be updated to 
    the now-current format. 
    
    If this is an object property, the contained objects will be asked 
    recursively to read themselves in from the same document. However, any 
    object that has the "file" attribute will read in its contents from that 
    file rather than from the supplied XML document, and the version number
    will be taken from that file rather than the argument supplied here. **/
    virtual void readFromXMLElement
       (SimTK::Xml::Element& propertyElement,
        int                  versionNumber) = 0;

    /** Output a serialized representation of this property by writing its
    value to the given XML property element. If the "use default value" 
    attribute is set for this property (meaning we don't have a meaningful 
    value for it) then you should not call this method unless you are trying 
    to serialize the defaults. Note that this method unconditionally 
    serializes the property; it does not check to see whether it should. 
    
    This method is not called for the special case of a one-object property,
    in which case only the object is written to the XML file (without the
    property element). In all other cases (simple property or property 
    containing an array of objects), the format is
    @code
        <propertyName> value(s) </propertyName>
    @endcode
    and that is the only format produced here since the empty-valued property
    element is supplied (with the property name as its tag). **/
    virtual void writeToXMLElement
       (SimTK::Xml::Element& propertyElement) const = 0;


    /** How may values are currently stored in this property? If this is an
    object property you can use this with getValueAsObject() to iterate over
    the contained objects. **/
    virtual int getNumValues() const = 0;

    /** If the concrete property allows it, clear the value list. **/
    virtual void clearValues() = 0;


    /** Return true if the given string is the XML tag name for one of the
    Object-derived types that is allowed by this property. If so, we expect that
    an element with that tag could be deserialized into a value element of this
    property. This always returns false for a simple property. **/
    virtual bool isAcceptableObjectTag
       (const std::string& objectTypeTag) const = 0;

    //--------------------------------------------------------------------------

private:
    void setNull();

    std::string _name;
    std::string _comment;
    bool        _valueIsDefault;    // current value is just the default

    int         _minListSize;       // minimum # values for property
    int         _maxListSize;       // maximum # value for property
};


}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_ABSTRACT_PROPERTY_H_
