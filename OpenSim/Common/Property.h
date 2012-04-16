#ifndef OPENSIM_PROPERTY2_H_
#define OPENSIM_PROPERTY2_H_
// Property.h
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
#include "Simbody.h"

#include <string>
#include <cmath>
#include <typeinfo>

namespace OpenSim {

//==============================================================================
//                                PROPERTY
//==============================================================================
// TODO: rename Property2 to Property

/** A %Property\<T> is a serializable (name, list-of-values) pair, where each 
value is of type T. The number of values allowed in the list is an attribute of
the property; most allow either just one or an arbitrary number, but some 
have other restrictions. 

Properties are maintained in a PropertyTable by %OpenSim's Object base class 
that is used for all serializable objects. Do not create %Property objects 
directly; instead, use the methods provided by the %OpenSim Object base class 
to create them:
@code
    Object::addProperty<T>()            // exactly one value required
    Object::addOptionalProperty<T>()    // zero or one value
    Object::addListProperty<T>()        // zero or more values
@endcode
These implicitly define the number of elements allowed as shown above. You 
can narrow the range for a list property by calling setAllowableListSize() on
the property after it is created, like this:
@code
    // Only 0,1,2 or 3 integer values allowed for this property's value list.
    addListProperty<int>("my_int_property", "a comment")
        .setAllowableListSize(0,3);
@endcode
(The above works because the addProperty() methods return a reference to the
newly created %Property.)

Type T must be a serializable type. Serializable types come in two categories:
  - simple types (like int or string) for which serialization instructions 
    have been provided, and
  - object types, in which case type T derives from Object and knows
    how to serialize itself.

When T is a simple type we'll write T=S and refer to a %Property\<S> as a 
"simple property". When T is an object type, we'll write T=O and
refer to a %Property\<O> as an "object property".

In case type O is a still-abstract Object-derived type like Function or 
Controller, a %Property\<O> can hold a mix of any concrete objects derived from 
O (e.g., any Object that can be dynamic_cast to a Function can be
held by a %Property\<Function>).

The objects in an object property will themselves have
properties so a %Property\<O> can be viewed as a node in the tree of objects
that constitute an %OpenSim Model. Simple properties %Property\<S> can be 
viewed as the terminal nodes of that tree. Properties are thus an integral part
of the structure of an %OpenSim Model; anything contained in a property is 
owned by that property; deleting the property deletes its contained objects. If
you want to \e reference another Object from within a property, use a string 
property to reference it by name; the result is a simple property. It is not
permitted for type T to be a pointer or reference.

<h3>XML file representation</h3>
The general representation for a %Property\<T> with name "prop_name" is
@code
    <prop_name> T T ... T </prop_name>
@endcode
where "T" is the XML representation for objects of type T. Note that if T is
an object type O, its representation follows the pattern
@code
    <OTypeName> OContents </OTypeName>
@endcode
where \c OTypeName stands for the name of the concrete, Object-derived class
being serialized, and \c OContents is the representation generated by that
class when asked to serialize itself.

A %Property\<O> that is restricted to holding <em>exactly one</em> object of 
type O is called a "one-object property". It could be represented in XML as
@code
    <prop_name> <OTypeName> OContents </OTypeName> </prop_name>
@endcode
but we allow a more compact representation for one-object properties:
@code
    <OTypeName name="prop_name"> OContents </OTypeName>
@endcode
In the one-object case it is also permissible for the property to be unnamed, 
in which case it may be referenced as though its name were the same as the 
object type name, and there is no separate "name" attribute. You can also give 
the object type name tag explicitly in Object::addProperty() as the property 
name but it is still considered unnamed. In either case the XML representation 
is just:
@code
    <OTypeName> OContents </OTypeName>
@endcode
On input, if a name attribute is seen for an unnamed property it is ignored; 
only the object type name tag matters in the unnamed case. Note that only 
one-object properties can be unnamed or named the same as the object type, and
no single OpenSim object can have more than one unnamed property of the 
same type.

<h3>%Property attributes</h3>

In addition to the name and list of values, every property has the following
attributes:
  - A comment string, provided at the time the property is created.
  - The minimum and maximum number of values allowed.
  - A "used default value" flag.

The "used default value" flag specifies that the value stored with this 
property was taken from a default object and not subsequently changed. A 
property with this flag set is not written out when a model is serialized. **/

template <class T>
class Property : public AbstractProperty {
public:
    // Default constructor, destructor, copy constructor, copy assignment

    /** Provides type-specific methods used to implement generic functionality.
    This class must be specialized for any 
    type T that is used in a Property\<T> instantiation, unless T is an 
    Object or something derived from Object. **/
    struct TypeHelper;

    /** Make a new, deep copy (clone) of this concrete property and return
    a pointer to the heap space. Caller must delete the returned object when
    done with it. **/
    Property* clone() const OVERRIDE_11 = 0;

    const T& operator[](int i) const {return getValue(i);}
    T& operator[](int i) {return updValue(i);}

    /** Return a const reference to the selected value from this property's 
    value list. If the property is at most single valued then the \a index is 
    optional and we'll behave as though index=0 were supplied.  **/
    const T& getValue(int index=-1) const {
        if (index < 0) {
            if (getMaxListSize()==1) index = 0;
            else throw OpenSim::Exception(
                "Property<T>::getValue(): an index must be "
                "provided for a property that takes a list "
                "of values.", __FILE__, __LINE__);
        }
        return getValueVirtual(index);
    }

    /** Return a writable reference to the selected value from this property's 
    value list. If the property is at most single valued then the \a index is 
    optional and we'll behave as though index=0 were supplied. **/
    T& updValue(int index=-1) {
        if (index < 0) {
            if (getMaxListSize()==1) index = 0;
            else throw OpenSim::Exception(
                "Property::updValue(): an index must be "
                "provided for a property that takes a list "
                "of values.", __FILE__, __LINE__);
        }
        return updValueVirtual(index); 
    }

    /** Add a copy of the supplied \a value to the end of this property's 
    value list. An exception is thrown if the property can't hold any more 
    values. The index assigned to this value is returned. **/
    int appendValue(const T& value) {
        if (getNumValues() >= getMaxListSize())
            throw OpenSim::Exception(
                "Property::appendValue(T&): property " + getName() 
                + " can't hold any more than " 
                + SimTK::String(getMaxListSize()) + " values.",
                __FILE__, __LINE__);
        return appendValueVirtual(value);
    }

    /** Add a copy of the supplied \a value to the end of this property's value 
    list. An exception is thrown if the property can't hold any more values. 
    The index assigned to this value is returned. Note that although we 
    accept a pointer here, we do not take over ownership. See **/
    int appendValue(const T* value) {
        if (value == NULL)
            throw OpenSim::Exception(
                "Property::appendValue(T*): null value not allowed.",
                __FILE__, __LINE__);
        if (getNumValues() >= getMaxListSize())
            throw OpenSim::Exception(
                "Property::appendValue(T*): property " + getName() 
                + " can't hold any more than "
                + SimTK::String(getMaxListSize()) + " values.",
                __FILE__, __LINE__);
        return appendValueVirtual(*value);
    }

    /** Add a new value to the end of this property's value list, taking over
    ownership of the supplied heap-allocated object. An exception
    is thrown if the property can't hold any more values. The index assigned
    to this value is returned. **/
    int adoptValue(T* value) {
        if (value == NULL)
            throw OpenSim::Exception(
                "Property::adoptValue(T*): null value not allowed.",
                __FILE__, __LINE__);
        if (getNumValues() >= getMaxListSize())
            throw OpenSim::Exception(
                "Property::adoptValue(T*): property " + getName() 
                + " can't hold any more than " + SimTK::String(getMaxListSize())
                + " values.");
        return appendHeapValueVirtual(value);
    }

    /** Return true if the given AbstractProperty references a concrete
    property of this type (%Property\<T>). Note that for this to return true,
    the type T must be exactly the type used when the concrete property was
    allocated; it is not sufficient for T to be a more general base type from
    which the actual type was derived. **/
    static bool isA(const AbstractProperty& prop) 
    {   return dynamic_cast<const Property*>(&prop) != NULL; }

    /** Attempt to downcast the given AbstractProperty to a concrete
    property of this type (%Property\<T>). An exception is thrown if
    this is not the right type; see isA() if you need to check first. **/
    static const Property& getAs(const AbstractProperty& prop) {
        const Property* p = dynamic_cast<const Property*>(&prop);
        if (p) return *p;
        throw OpenSim::Exception
           ("Property<T>::getAs(): Property " + prop.getName() 
            + " was not of type " + std::string(SimTK::NiceTypeName<T>::name()),
            __FILE__, __LINE__);
    }

    /** Attempt to downcast the given AbstractProperty to a writable concrete
    property of this type (%Property\<T>). An exception is thrown if
    this is not the right type; see isA() if you need to check first. **/
    static Property& updAs(AbstractProperty& prop) {
        Property* p = dynamic_cast<Property*>(&prop);
        if (p) return *p;
        throw OpenSim::Exception
           ("Property<T>::updAs(): Property " + prop.getName() 
            + " was not of type " + std::string(SimTK::NiceTypeName<T>::name()),
            __FILE__, __LINE__);
    }

protected:
    // This is the interface that SimpleProperty and ObjectProperty must
    // implement.
    virtual const T& getValueVirtual(int index) const = 0;
    virtual T& updValueVirtual(int index) = 0;
    virtual int appendValueVirtual(const T& value) = 0;
    virtual int adoptHeapValueVirtual(T* value) = 0;

};



//==============================================================================
//                    TYPE HELPER SPECIALIZATIONS
//==============================================================================

/** This is the generic definition of Property::TypeHelper to be used 
whenever T does not have a specialization, meaning that T must be a type 
derived from class Object. Any non-Object type that is to be used as a property 
type \e must provide a specialization of this class. **/
template <class T> struct Property<T>::TypeHelper {
    static ObjectProperty<T>* create(const std::string& name, bool isOne);
    static const char* name() {return "Obj";}
    static bool isEqual(const T& a, const T& b) {return a==b;}
};


template<> struct Property<bool>::TypeHelper {
    static SimpleProperty<bool>* create(const std::string& name, bool isOne);
    static const char* name() {return "bool";}
    static bool isEqual(bool a, bool b) {return a==b;}
};
template<> struct Property<int>::TypeHelper {
    static SimpleProperty<int>* create(const std::string& name, bool isOne);
    static const char* name() {return "int";}
    static bool isEqual(int a, int b) {return a==b;}
};
template<> struct Property<std::string>::TypeHelper {
    static SimpleProperty<std::string>* 
    create(const std::string& name, bool isOne);
    static void destructIfNeeded(std::string& value) {}
    static std::string& derefIfNeeded(std::string& value) {return value;}
    static const std::string& cloneIfNeeded(const std::string& value) 
    {   return value; }
    static const char* name() {return "string";}
    static bool isEqual(const std::string& a, const std::string& b)
    {   return a==b; }
};

// Floating point values' isEqual() operator returns true if all the numbers
// are equal to within a tolerance. We also say NaN==NaN, which is not standard
// IEEE floating point behavior.
template<> struct Property<double>::TypeHelper {
    static SimpleProperty<double>* create(const std::string& name, bool isOne);
    static const char* name() {return "double";}
    OSIMCOMMON_API static bool isEqual(double a, double b);
};
template<> struct Property<SimTK::Vec3>::TypeHelper  {
    static SimpleProperty<SimTK::Vec3>* 
    create(const std::string& name, bool isOne);
    static const char* name() {return "Vec3";}
    OSIMCOMMON_API static bool isEqual(const SimTK::Vec3& a, 
                                       const SimTK::Vec3& b);
};
template<> struct Property<SimTK::Vector>::TypeHelper  {
    static SimpleProperty<SimTK::Vector>* 
    create(const std::string& name, bool isOne);
    static const char* name() {return "Vector";}
    OSIMCOMMON_API static bool isEqual(const SimTK::Vector& a, 
                                       const SimTK::Vector& b);
};
template<> struct Property<SimTK::Transform>::TypeHelper  {
    static SimpleProperty<SimTK::Transform>* 
    create(const std::string& name, bool isOne);
    static const char* name() {return "Transform";}
    OSIMCOMMON_API static bool isEqual(const SimTK::Transform& a, 
                                       const SimTK::Transform& b);
};


//==============================================================================
//                             SIMPLE PROPERTY
//==============================================================================
/** This subclass of Property<T> is used when type T=S is a "simple" type, 
meaning it is not derived from Object. **/
template <class T>
class SimpleProperty : public Property<T> {
public:
    /** A simple property must have a non-null name. **/
    SimpleProperty(const std::string& name, bool isOneValue) { 
        if (name.empty())
            throw OpenSim::Exception(
                "addProperty<" + std::string(SimTK::NiceTypeName<T>::name())
                + ">(): a simple (non-Object) property must have a name.");        
        this->setName(name); 
        if (isOneValue) this->setAllowableListSize(1); 
    }

    // Default destructor, copy constructor, copy assignment.

    SimpleProperty* clone() const FINAL_11 
    {   return new SimpleProperty(*this); }
   
    std::string toString() const                        FINAL_11 {
        std::stringstream out;
        if (!this->isOneValueProperty()) out << "(";
        writeSimplePropertyToStream(out);
        if (!this->isOneValueProperty()) out << ")";
        return out.str();
    }

    bool isUnnamedProperty() const FINAL_11 {return false;}
    bool isObjectProperty() const FINAL_11 {return false;}
    bool isAcceptableObjectTag(const std::string&) const FINAL_11 
    {   return false; }

    int getNumValues() const FINAL_11 {return values.size(); }
    void clearValues() FINAL_11 {values.clear();}

    bool isEqualTo(const AbstractProperty& other) const FINAL_11 {
        // Check here rather than in base class because the old
        // Property_Deprecated implementation can't copy this flag right.
        if (this->getUseDefault() != other.getUseDefault())
            return false;
        assert(size() == other.size()); // base class checked
        const SimpleProperty& otherS = SimpleProperty::getAs(other);
        for (int i=0; i<values.size(); ++i)
            if (!TypeHelper::isEqual(values[i], otherS.values[i]))
                return false;
        return true;
    }

    // Property element is a value element, so the values is just a single
    // string. We'll divide into blank-separated tokens and then parse
    // into elements of type T.
    void readFromXMLElement
       (SimTK::Xml::Element& propertyElement,
        int                  versionNumber) FINAL_11 {
        std::istringstream valstream(propertyElement.getValue());
        if (!readSimplePropertyFromStream(valstream)) {
            std::cerr << "Failed to read " << SimTK::NiceTypeName<T>::name()
            << " property " << this->getName() << "; input='" 
            << valstream.str().substr(0,50) // limit displayed length
            << "'.\n";
        }
        if (values.size() < this->getMinListSize()) {
            std::cerr << "Not enough values for " 
            << SimTK::NiceTypeName<T>::name() << " property " << this->getName() 
            << "; input='" << valstream.str().substr(0,50) // limit displayed length 
            << "'. Expected " << this->getMinListSize()
            << ", got " << values.size() << ".\n";
        }
        if (values.size() > this->getMaxListSize()) {
            std::cerr << "Too many values for " 
            << SimTK::NiceTypeName<T>::name() << " property " << this->getName() 
            << "; input='" << valstream.str().substr(0,50) // limit displayed length 
            << ". Expected " << this->getMaxListSize()
            << ", got " << values.size() << ". Ignoring extras.\n";

            values.resize(this->getMaxListSize());
        }
    }

    // Property element will be just a value element. We'll serialize it
    // using an unformatted write to produce a series of blank-separated 
    // tokens.
    void writeToXMLElement
       (SimTK::Xml::Element& propertyElement) const FINAL_11 {
        std::ostringstream valstream;
        writeSimplePropertyToStream(valstream);
        propertyElement.setValue(valstream.str()); 
    } 


    const Object& getValueAsObject(int index=-1) const FINAL_11 {
        throw OpenSim::Exception(
                "SimpleProperty<T>::getValueAsObject(): property " 
                + this->getName() + " is not an Object property."); 
    }

    Object& updValueAsObject(int index=-1) FINAL_11 {
        throw OpenSim::Exception(
                "SimpleProperty<T>::updValueAsObject(): property " 
                + this->getName() + " is not an Object property."); 
    }

    void setValueAsObject(const Object& obj, int index=-1) FINAL_11 {
        throw OpenSim::Exception(
                "SimpleProperty<T>::setValueAsObject(): property " 
                + this->getName() + " is not an Object property."); 
    }

    // This is the Property<T> interface implementation.



    static bool isA(const AbstractProperty& prop) 
    {   return dynamic_cast<const SimpleProperty*>(&prop) != NULL; }

    static const SimpleProperty& getAs(const AbstractProperty& prop) {
        const SimpleProperty* p = dynamic_cast<const SimpleProperty*>(&prop);
        if (p) return *p;
        throw OpenSim::Exception
           ("SimpleProperty<T>::getAs(): Property " + prop.getName() 
            + " was not of simple type "
            + std::string(SimTK::NiceTypeName<T>::name()),
            __FILE__, __LINE__);
    }

    static SimpleProperty& updAs(AbstractProperty& prop) {
        SimpleProperty* p = dynamic_cast<SimpleProperty*>(&prop);
        if (p) return *p;
        throw OpenSim::Exception
           ("SimpleProperty<T>::updAs(): Property " + prop.getName() 
           + " was not of simple type "
            + std::string(SimTK::NiceTypeName<T>::name()),
            __FILE__, __LINE__);
    }

private:
    // Base class checks the index.
    const T& getValueVirtual(int index) const   FINAL_11 
    {   return values[index]; }
    T& updValueVirtual(int index)               FINAL_11 
    {   return values[index]; }
    int appendValueVirtual(const T& value)     FINAL_11
    {   values.push_back(value); return values.size()-1; }
    int adoptHeapValueVirtual(T* valuep)     FINAL_11
    {   values.push_back(*valuep); // make a copy
        delete valuep; // throw out the old one
        return values.size()-1; }

    // This is the default implementation; specialization is required if
    // the Simbody default behavior is different than OpenSim's; e.g. for
    // Transform serialization.
    bool readSimplePropertyFromStream(std::istream& in) {
        return SimTK::readUnformatted(in, values);
    }

    // This is the default implementation; specialization is required if
    // the Simbody default behavior is different than OpenSim's; e.g. for
    // Transform serialization.
    void writeSimplePropertyToStream(std::ostream& o) const {
        SimTK::writeUnformatted(o, values);
    }
    SimTK::Array_<T,int> values;
};

// We have to provide specializations for Transform because read/write
// unformatted would not know to convert to X-Y-Z body fixed Euler angles
// followed by the position vector.

template <> bool SimpleProperty<SimTK::Transform>::
readSimplePropertyFromStream(std::istream& in)
{   
    SimTK::Array_<SimTK::Vec6,int> rotTrans;
    values.clear();
    if (!SimTK::readUnformatted(in, rotTrans)) return false;

    for (int i=0; i<rotTrans.size(); ++i) {
        SimTK::Transform X;
        const SimTK::Vec3& angles = rotTrans[i].getSubVec<3>(0);
        const SimTK::Vec3& pos = rotTrans[i].getSubVec<3>(3);
        X.updR().setRotationToBodyFixedXYZ(angles);
        X.updP() = pos;
        values.push_back(X);
    }
    return true;
}

template <> void SimpleProperty<SimTK::Transform>::
writeSimplePropertyToStream(std::ostream& o) const
{   
    SimTK::Array_<SimTK::Vec6> rotTrans;
    for (int i=0; i<values.size(); ++i) {
        SimTK::Vec6 X6;
        SimTK::Vec3& angles = X6.updSubVec<3>(0);
        SimTK::Vec3& pos    = X6.updSubVec<3>(3);
        angles = values[i].R().convertRotationToBodyFixedXYZ();
        pos = values[i].p();
        rotTrans.push_back(X6);
    }    
    SimTK::writeUnformatted(o, rotTrans);
}




//==============================================================================
//                             OBJECT PROPERTY
//==============================================================================
/** This subclass of Property<T> is used when type T=O is an "object" type, 
meaning it is derived from %OpenSim's Object serializable base class. **/
template <class T>
class ObjectProperty : public Property<T> {
public:
    /** A one-object property can be unnamed, in which case we use the 
    object type tag as a name for lookup purposes. We consider the property
    unnamed if its name is null or if the object type tag is given explicitly
    as its name. **/
    ObjectProperty(const std::string& name, bool isOneValue) 
    :   isUnnamed(false) {
        objectClassName = T::getClassName();
        if (name.empty() || name == objectClassName) {
            if (!isOneValue) 
                throw OpenSim::Exception(
                    "addProperty<" + objectClassName + 
                    ">(): only a one-object property can be unnamed or use "
                    "the object type as a name.");

            isUnnamed = true;
            this->setName(objectClassName); // use object type as property name
        } else
            this->setName(name);

        if (isOneValue) this->setAllowableListSize(1);
    }

    // Default destructor, copy constructor, copy assignment

    const std::string& getObjectClassName() const {return objectClassName;}


    ObjectProperty* clone() const FINAL_11 
    {   return new ObjectProperty(*this); }

    std::string toString() const                        FINAL_11 {
        if (objects.empty()) return "(No Objects)";
        std::string out;
        if (!this->isOneValueProperty()) out += '(';
        for (int i=0; i < objects.size(); ++i) {
            if (i != 0) out += ' ';
            out += objects[i]->getConcreteClassName();
        }
        if (!this->isOneValueProperty()) out += ')';
        return out;
    }

    bool isUnnamedProperty() const FINAL_11 {return isUnnamed;}
    bool isObjectProperty() const FINAL_11 {return true;}
    bool isAcceptableObjectTag(const std::string& objectTypeTag) const FINAL_11 
    {   return Object::isObjectTypeDerivedFrom<T>(objectTypeTag); }

    int getNumValues() const FINAL_11 {return objects.size();}
    void clearValues() FINAL_11 {objects.clear();}

    bool isEqualTo(const AbstractProperty& other) const FINAL_11 {
        // Check here rather than in base class because the old
        // Property_Deprecated implementation can't copy this flag right.
        if (this->getUseDefault() != other.getUseDefault())
            return false;
        assert(size() == other.size()); // base class checked
        const ObjectProperty& otherO = ObjectProperty::getAs(other);
        for (int i=0; i<objects.size(); ++i)
            if (!(objects[i] == otherO.objects[i]))
                return false;
        return true;
    }

    // Property element is a compound element, consisting of subelements
    // each of which is one of the object values.
    void readFromXMLElement
       (SimTK::Xml::Element& propertyElement,
        int                  versionNumber) FINAL_11
    {
        clearValues();
		// LOOP THROUGH PROPERTY ELEMENT'S CHILD ELEMENTS
        // Each element is expected to be an Object of some type given
        // by the element's tag; that type must be derived from O or we
        // can't store it in this property.
		int objectsFound = 0;
		SimTK::Xml::element_iterator iter = propertyElement.element_begin();
		for (; iter != propertyElement.element_end(); ++iter) {
            const SimTK::String& objTypeTag = iter->getElementTag();

            if (!Object::isObjectTypeDerivedFrom<T>(objTypeTag)) {
                std::cerr << "Object type " << objTypeTag  
                          << " wrong for " << objectClassName
                          << " property " << this->getName()
                          << "; ignoring.\n";
                continue;                        
            }
			++objectsFound;

            if (objectsFound > this->getMaxListSize())
                continue; // ignore this one

			// Create an Object of the element tag's type.
			Object* object = Object::newInstanceOfType(objTypeTag);
            assert(object); // we just checked above
			object->updateFromXMLNode(*iter, versionNumber);

            T* objectT = dynamic_cast<T*>(object);
            assert(objectT); // should have worked by construction
            adoptHeapValueVirtual(objectT); // don't copy
		}

        if (objectsFound < this->getMinListSize()) {
            std::cerr << "Got " << objectsFound 
                      << " object values for Property "
                      << this->getName() << " but the minimum is " 
                      << this->getMinListSize() << ". Continuing anyway.\n"; 
        }
        if (objectsFound > this->getMaxListSize()) {
            std::cerr << "Got " << objectsFound
                      << " object values for Property "
                      << this->getName() << " but the maximum is " 
                      << this->getMaxListSize() << ". Ignoring the rest.\n"; 
        }
    }

    // Each object value serializes itself into a subelement of the given
    // property element.
    void writeToXMLElement
       (SimTK::Xml::Element& propertyElement) const FINAL_11 
    {
        for (int i=0; i < objects.size(); ++i)
            const_cast<T&>(*objects[i]).updateXMLNode(propertyElement);
    }

    const Object& getValueAsObject(int index=-1) const FINAL_11 {
        if (index < 0 && this->getMinListSize()==1 && this->getMaxListSize()==1)
            index = 0;
        return *objects[index];
    }

    Object& updValueAsObject(int index=-1) FINAL_11 {
        if (index < 0 && this->getMinListSize()==1 && this->getMaxListSize()==1)
            index = 0;
        return *objects[index];
    }

    void setValueAsObject(const Object& obj, int index=-1) FINAL_11 {
        if (index < 0 && this->getMinListSize()==1 && this->getMaxListSize()==1)
            index = 0;
        T* newObjT = dynamic_cast<T*>(obj.clone());
        if (newObjT == NULL) 
            throw OpenSim::Exception
               ("ObjectProperty<T>::setValueAsObject(): the supplied object"
                + obj.getName() + " was of type " + obj.getConcreteClassName()
                + " which can't be stored in this " + objectClassName
                + " property " + this->getName(),
                __FILE__, __LINE__);

        objects[index] = newObjT;
    }

    static bool isA(const AbstractProperty& prop) 
    {   return dynamic_cast<const ObjectProperty*>(&prop) != NULL; }

    static const ObjectProperty& getAs(const AbstractProperty& prop) {
        const ObjectProperty* p = dynamic_cast<const ObjectProperty*>(&prop);
        if (p) return *p;
        throw OpenSim::Exception
           ("ObjectProperty<T>::getAs(): Property " + prop.getName() 
            + " was not of object type " + T::getClassName(), 
            __FILE__, __LINE__);
    }

    static ObjectProperty& updAs(AbstractProperty& prop) {
        ObjectProperty* p = dynamic_cast<ObjectProperty*>(&prop);
        if (p) return *p;
        throw OpenSim::Exception
           ("ObjectProperty<T>::updAs(): Property " + prop.getName() 
           + " was not of object type " + T::getClassName(), 
            __FILE__, __LINE__);
    }
private:
    // Base class checks the index.
    const T& getValueVirtual(int index) const   FINAL_11 
    {   return *objects[index]; }
    T& updValueVirtual(int index)               FINAL_11 
    {   return *objects[index]; }
    int appendValueVirtual(const T& obj)        FINAL_11
    {   objects.push_back();        // add empty element
        objects.back() = obj;       // insert a copy
        return objects.size()-1; }
    int adoptHeapValueVirtual(T* objp)         FINAL_11
    {   objects.push_back();        // add empty element
        objects.back().reset(objp); // take over ownership
        return objects.size()-1; }

    std::string  objectClassName;
    bool         isUnnamed;    // we'll use the objectTypeTag as a name                    
    SimTK::Array_<SimTK::ClonePtr<T>,int> objects;
};


//==============================================================================
//                        TYPE HELPER IMPLEMENTATION
//==============================================================================
// These had to wait for SimpleProperty and ObjectProperty.

template <class T> 
inline ObjectProperty<T>* Property<T>::
TypeHelper::create(const std::string& name, bool isOne) 
{   return new ObjectProperty<T>(name, isOne); }

inline SimpleProperty<bool>* Property<bool>::
TypeHelper::create(const std::string& name, bool isOne) 
{   return new SimpleProperty<bool>(name, isOne); }

inline SimpleProperty<int>* Property<int>::
TypeHelper::create(const std::string& name, bool isOne) 
{   return new SimpleProperty<int>(name, isOne); }

inline SimpleProperty<std::string>* Property<std::string>::
TypeHelper::create(const std::string& name, bool isOne) 
{   return new SimpleProperty<std::string>(name, isOne); }

inline SimpleProperty<double>* Property<double>::
TypeHelper::create(const std::string& name, bool isOne) 
{   return new SimpleProperty<double>(name, isOne); }

inline SimpleProperty<SimTK::Vec3>* Property<SimTK::Vec3>::
TypeHelper::create(const std::string& name, bool isOne) 
{   return new SimpleProperty<SimTK::Vec3>(name, isOne); }

inline SimpleProperty<SimTK::Vector>* Property<SimTK::Vector>::
TypeHelper::create(const std::string& name, bool isOne) 
{   return new SimpleProperty<SimTK::Vector>(name, isOne); }

inline SimpleProperty<SimTK::Transform>* Property<SimTK::Transform>::
TypeHelper::create(const std::string& name, bool isOne) 
{   return new SimpleProperty<SimTK::Transform>(name, isOne); }

}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_PROPERTY2_H_
