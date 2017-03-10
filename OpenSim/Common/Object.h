#ifndef OPENSIM_OBJECT_H_
#define OPENSIM_OBJECT_H_
/* -------------------------------------------------------------------------- *
 *                             OpenSim:  Object.h                             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson, Ayman Habib, Ajay Seth, Michael A. Sherman   *
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */

#ifdef _WIN32
#pragma warning( disable : 4251 )
#pragma warning( disable : 4786 )
#pragma warning( disable : 4660 )
#endif

// INCLUDES

#include "osimCommonDLL.h"
#include "PropertySet.h"
#include "PropertyTable.h"
#include "Property.h"

#include <cstring>
#include <cassert>

// DISABLES MULTIPLE INSTANTIATION WARNINGS


// EXPORT LINE FOR MICROSOFT VISUAL C++
#ifdef _WIN32
#ifndef SWIG
template class OSIMCOMMON_API OpenSim::ArrayPtrs<OpenSim::Object>;
#endif
#endif


#ifdef SWIG
    #ifdef OSIMCOMMON_API
        #undef OSIMCOMMON_API
    #endif
    #define OSIMCOMMON_API

    #ifdef SWIGJAVA
        #define SWIG_DECLARE_EXCEPTION throw(OpenSim::Exception)
    #else
        #define SWIG_DECLARE_EXCEPTION
    #endif
#else
    #define SWIG_DECLARE_EXCEPTION
#endif

// Forward-declare SimTK types.
namespace SimTK {
    // Needed for Object_GetClassName<SimTK::SpatialVec>, defined in this file.
    typedef Vec<2, Vec3> SpatialVec;
}

namespace OpenSim { 

// CONSTANTS
const char ObjectDEFAULT_NAME[] = "default";

class XMLDocument;

//==============================================================================
//                                 OBJECT
//==============================================================================
/** This is the base class for all %OpenSim objects that are serializable 
(meaning they can be written to and read back from files). In particular, all 
ModelComponent objects derive from %Object. It provides a common base class 
from which to derive serializable objects and also some basic functionality, 
such as writing to files in XML format, managing properties, and the equality, 
less than, and output operators.

An %Object maintains a table of "properties" that know how to read themselves 
from XML and write themselves to XML. The available Property types are
  -# Primitive data types (int, bool, double, std::string, ...) 
  -# Properties that contain other Objects, 
  -# Properties containing lists of either of the previous 2 categories

It is important to note that Objects and Properties together form a recursive
tree structure that is the representation of an %OpenSim Model. See the
documentation for the OpenSim::Property class for more information.

<h3>%Object declaration</h3>

The declaration of every class derived from %Object \e must have its first line
(that is, immediately after the "{" in the class declaration) one of four
standard "boilerplate" macros:
@code
  OpenSim_DECLARE_CONCRETE_OBJECT  (ClassName, SuperclassName);
  OpenSim_DECLARE_CONCRETE_OBJECT_T(ClassName, T, SuperclassName);
  OpenSim_DECLARE_ABSTRACT_OBJECT  (ClassName, SuperclassName);
  OpenSim_DECLARE_ABSTRACT_OBJECT_T(ClassName, T, SuperclassName);
@endcode
("Superclass" means the immediate class from which the class derives; that
terminology is borrowed from Java. It is often called the "Parent" class but 
we'll use "Super" which is more precise.) The "_T" variants of the above macros 
are used for objects that are templatized, like Set\<T>. 

These macros provide a standardized set of declarations for every object, 
including
@code
    typedef ClassName      Self;                // for all classes
    typedef SuperclassName Super;               // for all classes
    static const std::string& getClassName();   // for all classes
    const std::string& getConcreteClassName();  // for concrete classes only
    ClassName* clone() const;                   // see below
@endcode
getClassName() is a static method that returns the name of the %Object-derived 
class for which it is invoked. For example, ModelComponent::getClassName() 
returns "ModelComponent". In contrast, getConcreteClassName() is a pure virtual 
method of %Object that returns the class name of the actual concrete object 
being referenced through the abstract base class. This method is implemented 
only in concrete classes.

Note that getClassName() and getConcreteClassName() will return the same string
only if the referenced class is concrete. For example,
@code
    Function* funcp = new LinearFunction(...);
    std::cout << funcp->getClassName();          // output: "Function"
    std::cout << funcp->getConcreteClassName();  // output: "LinearFunction"
@endcode

For concrete objects, the class name is used as the "object type tag", the tag 
string that will appear in XML files. Also, when a Property\<T> has no name
(allowed for properties that contain just a single object) the object class 
name T (which may be abstract like Function or ModelComponent) is used to 
select the property. See OpenSim::Property for more information.

The standard clone() method produces a duplicate of a concrete object and thus
is implemented only for concrete classes. However, the return type must
always match the type of the invoking object (this is called a "covariant type"
and does not change the method's identity). It is therefore redeclared even in
abstract classes, but remains pure virtual in those cases. That means if you
invoke Function::clone() you'll get back a Function* rather than an Object*;
this avoids many unnecessary invocations of the awkward and expensive
dynamic_cast operator.

<h3>%Object registration and renaming</h3>

An %Object type needs to be "registered" by calling Object::registerType() with
an instance of a concrete object so that the serialization infrastructure knows 
what kind of %Object to create when it encounters a specific XML tag. This 
associates the concrete object's class name (object type tag) with a default 
instance of that object. The registration process is normally done during 
dynamic library (DLL) loading, that is, as part of the static initializer
execution that occurs before program execution.

For backwards compatibility, we support a renaming mechanism in which 
now-deprecated class names can be mapped to their current equivalents. This
is done via a string-to-string table mapping the old names to the new ones;
only the current names appear in the registered objects table. Specification of
these aliases is done immediately after registration in the DLL static 
initializer.

<h3>Defaults mechanism</h3>

When an %Object is registered (either programmatically, or
overridden in the defaults section of a document), a copy of it is maintained 
in a dictionary as a "default" object of its class. When new instances of this 
class are requested, the contents of the default object are used to populate the
new instance before deserialization. This allows for specifying default values
that will be commonly used in one place in the XML file rather than with each 
object which leads to smaller files that are easier to read. Property values
that obtain their values from the defaults and are not subsequently overridden
are marked as being default values, allowing us to avoid writing
them back out when serializing.

@author Frank C. Anderson, Ayman Habib, Ajay Seth, Michael Sherman 
@see OpenSim::Property
**/
class OSIMCOMMON_API Object  
{
//------------------------------------------------------------------------------
// PUBLIC METHODS
//------------------------------------------------------------------------------
public:
    // Constructors are protected.


    /**
     * Virtual destructor for cleanup
     */
    virtual ~Object();

    /** Create a new heap-allocated copy of the concrete object to which this 
    %Object refers. It is up to the caller to delete the returned object
    when no longer needed. Every concrete object deriving from %Object 
    implements this pure virtual method automatically, via the declaration
    macro it invokes (e.g., OpenSim_DECLARE_CONCRETE_OBJECT()). Note that the 
    concrete class overrides modify the return type to be a pointer to the
    \e concrete object; that still overrides the base class method because the 
    return type is covariant with (that is, derives from) %Object. **/
    virtual Object* clone() const = 0;

    /** Returns the class name of the concrete %Object-derived class of the
    actual object referenced by this %Object, as a string. This is the 
    string that is used as the tag for this concrete object in an XML file.
    Every concrete class derived from %Object automatically overrides this
    method via the declaration macro it uses. See getClassName() to get the 
    class name of the referencing (possibly abstract) class rather than the 
    concrete object.
    @see getClassName() **/
    virtual const std::string& getConcreteClassName() const = 0;

    /// @cond
    // This is an assignment operator for use in Java.
    virtual void assign(Object &aObject) = 0;
    /// @endcond

    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
    /**
     * Equality operator wrapper for use from languages not supporting operator
     * overloading.
     */
    bool isEqualTo(const Object &aObject) const
    {
        return ((*this)==aObject);
    }

    #ifndef SWIG
    /** Copy assignment copies he base class fields, including the 
    properties. **/
    Object& operator=(const Object &aObject);
    /** Determine if two objects are equal. They are equal if all the simple
    base class members are equal, both objects have the same number of 
    properties and corresponding properties are equal, and if the objects
    are the same concrete type and the concrete class says they are equal. 
    Concrete object classes must override this if they have any fields to
    compare, but be sure to invoke the base class operator too. **/
    virtual bool operator==(const Object &aObject) const;
    /** Provide an ordering for objects so they can be put in sorted
    containers. **/
    virtual bool operator<(const Object &aObject) const;
    /** Write the type and name of this object into the given output stream. **/
    friend std::ostream& operator<<(std::ostream &aOut,
                                    const Object &aObject) {
        aOut << aObject.getConcreteClassName() << " " << aObject.getName();
        return(aOut);
    };
    #endif

    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------
    /** %Set the name of the Object. */
    void setName(const std::string& name);
    /** Get the name of this Object. */
    const std::string& getName() const;
    /** %Set description, a one-liner summary. */
    void setDescription(const std::string& description);
    /** Get description, a one-liner summary. */
    const std::string& getDescription() const;

    /** Get Authors of this Object */
    const std::string& getAuthors() const { return _authors; };
    /** %Set Authors of this object. Call this method in your constructor if needed. */
    void setAuthors(const std::string& authors) { _authors=authors; };

    /** Get references or publications to cite if using this object. */
    const std::string& getReferences() const { return _references; };
    /** %Set references or publications to cite if using this object. */
    void setReferences(const std::string& references) 
    {   _references=references; };


    //--------------------------------------------------------------------------
    // PUBLIC ACCESS TO PROPERTIES
    //--------------------------------------------------------------------------
    /** @name              Public access to properties
    Methods in this section are for public access to the properties maintained
    by this OpenSim %Object. Properties are normally accessed through methods
    of the concrete %Object-derived classes that are generated by the 
    Property declaration macros; see OpenSim::Property for information.
    However, when dealing with Objects from "the outside", as is done in the
    GUI, these methods allow access to properties via the property 
    base class AbstractProperty to  support various type-independent property 
    services. That is particularly useful for %Object-containing properties
    since the objects can be obtained without knowing their concrete types.
    For simple types (e.g. int, std::string) you can only obtain the values if
    you know the expected type. For those types, or when you know the 
    expected %Object type, you can use the templatized methods to deal with 
    the concrete values. **/
    /**@{**/
    /** Determine how many properties are stored with this %Object. These
    are numbered 0..n-1 in the order they were created. **/
    // Note: new properties come first, deprecated ones afterwards.
    int getNumProperties() const;
    /** Get a const reference to a property by its index number, returned as
    an AbstractProperty. **/
    const AbstractProperty& getPropertyByIndex(int propertyIndex) const;
    /** Get a writable reference to a property by its index number, returned as
    an AbstractProperty. **/
    AbstractProperty& updPropertyByIndex(int propertyIndex);

    /** Return true if this %Object has a property of any type with the 
    given \a name, which must not be empty. **/
    bool hasProperty(const std::string& name) const;
    /** Get a const reference to a property by its name, returned as
    an AbstractProperty. An exception is thrown if no property by this name
    is present in this %Object. **/
    const AbstractProperty& getPropertyByName(const std::string& name) const;
    /** Get a writable reference to a property by its name, returned as
    an AbstractProperty. An exception is thrown if no property by this name
    is present in this %Object. **/
    AbstractProperty& updPropertyByName(const std::string& name);

    /** Return true if this %Object contains an unnamed, one-object property
    that contains objects of the given template type T. The type must match
    exactly the type used when this property was created with 
    addProperty<T>(). **/
    template <class T> bool hasProperty() const;

    /** Get property of known type Property\<T> as a const reference; 
    the property must be present and have the right type. This is primarily
    used by the Property declaration macros for fast access to properties. **/
    template <class T> const Property<T>& 
    getProperty(const PropertyIndex& index) const;

    /** Get property of known type Property\<T> as a writable reference;
    the property must be present and have the right type. This is primarily
    used by the Property declaration macros for fast access to properties. **/
    template <class T> Property<T>& 
    updProperty(const PropertyIndex& index);

    /** Returns \c true if no property's value has changed since the last time
    setObjectIsUpToDateWithProperties() was called. **/
    bool isObjectUpToDateWithProperties() const {return _objectIsUpToDate;}

    /** Dump formatted property information to a given output stream, useful
    for creating a "help" facility for registered objects. Object name, 
    property name, and property comment are output. Input is a
    class name and property name. If the property name is the empty string or
    just "*", then information for all properties in the class is printed. If 
    the class name is empty, information in all properties of all registered 
    classes is printed.
    @param          os 
        Output stream to which info is printed.
    @param          classNameDotPropertyName 
        A string combining the class name and property name. The two names 
        should be separated by a period (ClassName.PropertyName). If 
        PropertyName is empty or "*", the information for all properties in the 
        class is printed. If ClassName is empty, the information for the 
        properties of all registered classes is printed.
    @param           printFlagInfo 
        Print to the ostream some instructions for using the -PropertyInfo
        command line flag.

    Returns false if the provided names do not match known classes or
    properties; otherwise, returns true. **/
    static bool PrintPropertyInfo(std::ostream&      os,
                                  const std::string& classNameDotPropertyName,
                                  bool printFlagInfo = true);
    /** Same as the other signature but the class name and property name are
    provided as two separate strings. 
    Returns false if the provided names do not match known classes or
    properties; otherwise, returns true. **/
    static bool PrintPropertyInfo(std::ostream&         os,
                                  const std::string&    className,
                                  const std::string&    propertyName,
                                  bool printFlagInfo = true);
    /**@}**/
    //--------------------------------------------------------------------------


    //--------------------------------------------------------------------------
    // REGISTRATION OF TYPES AND DEFAULT OBJECTS
    //--------------------------------------------------------------------------
    /** @name          Registration of types and default objects
    Methods in this section deal with the requirement that all %OpenSim 
    types derived from %Object must be registered and a default instance
    provided. This enables reading these objects from XML files. You can also
    recognize now-obsolete names for objects and have them quietly mapped to
    their modern names using the renameType() method. Rename can also be used
    programmatically to replace one registered type with another, because
    renaming occurs prior to object lookup. **/
    /**@{**/

    /** Register an instance of a class; if the class is already registered it
    will be replaced. This is normally called as part of the static
    initialization of a dynamic library (DLL). The supplied object's concrete
    class name will be used as a key, and a \e copy (via clone()) of the 
    supplied %Object is used as the default value for objects of this type when 
    created (typically during the deserialization process when reading an 
    XML file). **/
    static void registerType(const Object& defaultObject);

    /** Support versioning by associating the current %Object type with an 
    old name. This is only allowed if \a newTypeName has already been 
    registered with registerType(). Renaming is applied first prior to lookup
    so can be used both for translating now-obsolete names to their new names
    and for overriding one registered type with another. **/
    static void renameType(const std::string& oldTypeName, 
                           const std::string& newTypeName);

    /** Return a pointer to the default instance of the registered (concrete)
    %Object whose class name is given, or NULL if the type is not registered.
    Note that this refers to the default %Object instance that is stored with
    the %Object class; do not delete it! If you want a copy of this object
    instead, use newInstanceOfType(). The given \a concreteClassName will be
    mapped through the renamed type table if necessary but the returned object
    will always have the new type name, which may differ from the supplied 
    one. Note that renaming is applied first, prior to looking up the name
    in the registered objects table.
    @see registerType(), renameType() **/
    static const Object* 
    getDefaultInstanceOfType(const std::string& concreteClassName);

    /** Return true if the given concrete object type represents a subclass of 
    the template object type T, and thus could be referenced with a T*. The 
    object type to be tested is given by its class name as a string. 
    For this to work the name must represent an already-registered object
    type. If necessary \a concreteClassName will be mapped through the renamed
    type table, so we'll return true if the class it maps to satisfies the
    condition. Note that renaming is applied first, prior to looking up the name
    in the registered objects table.
    @see registerType(), renameType() **/
    template <class T> static bool
    isObjectTypeDerivedFrom(const std::string& concreteClassName) {
        const Object* defObj = getDefaultInstanceOfType(concreteClassName);
        if (defObj == NULL) return false;
        return dynamic_cast<const T*>(defObj) != NULL;
    }

    /** Create a new instance of the concrete %Object type whose class name is 
    given as \a concreteClassName. The instance is initialized to the default 
    object of corresponding type, possibly after renaming to the current class 
    name. Writes a message to stderr and returns null if the tag isn't 
    registered. **/
    static Object* newInstanceOfType(const std::string& concreteClassName);

    /** Retrieve all the typenames registered so far. This is done by traversing
    the registered objects map, so only concrete classes that have registered 
    instances are returned; renamed types will not appear unless they were
    separately registered. (Note that even if one registered type has been
    renamed to another, both will appear in the returned list.) The result 
    returned in \a typeNames should not be cached while more shared libraries 
    or plugins are loaded, because more types may be registered as a result. 
    Instead the list should be reconstructed whenever in doubt. **/
    static void getRegisteredTypenames(Array<std::string>& typeNames);

    /** Return an array of pointers to the default instances of all registered
    (concrete) %Object types that derive from a given %Object-derived type 
    that does not have to be concrete. This is useful, for example, to find 
    all Joints, Constraints, ModelComponents, Analyses, etc. **/
    template<class T> static void 
    getRegisteredObjectsOfGivenType(ArrayPtrs<T>& rArray) {
        rArray.setSize(0);
        rArray.setMemoryOwner(false);
        for(int i=0; i<_registeredTypes.getSize(); i++) {
            T* obj = dynamic_cast<T*>(_registeredTypes[i]);
            if (obj) rArray.append(obj);
        }
    }
    /**@}**/

    //--------------------------------------------------------------------------
    // XML
    //--------------------------------------------------------------------------
    /** @name                  XML reading and writing
    These methods deal with writing out in-memory objects to XML files
    (serializing) and reading XML files to reconstruct in-memory objects
    (deserializing). **/
    /**@{**/
    /** Create an %OpenSim object whose type is based on the tag at the root 
    node of the XML file passed in. This is useful since the constructor of 
    %Object doesn't have the proper type info. This works by using the defaults 
    table so that %Object does not need to know about its derived classes. It 
    uses the defaults table to get an instance. **/
    static Object* makeObjectFromFile(const std::string& fileName);

    /** We're given an XML element from which we are to populate this %Object.
    If the element has a \c file attribute, we'll instead read the %Object from
    that file. Otherwise we'll invoke updateFromXMLNode() to read the %Object
    directly from the supplied element. Note that a relative file name will
    be interpreted relative to the current working directory, but that will
    normally have been set earlier to the directory containing the top-level 
    (root) %Object, such as the Model file. **/
    void readObjectFromXMLNodeOrFile
       (SimTK::Xml::Element& objectElement, 
        int                  versionNumber);

    /** Use this method to deserialize an object from a SimTK::Xml::Element. The 
    element is assumed to be in the format consistent with the passed-in 
    \a versionNumber. If there is a file attribute in \a objectElement it
    will be ignored; if you want it processed you should call 
    readObjectFromXMLNodeOrFile() instead. **/
    virtual void updateFromXMLNode(SimTK::Xml::Element& objectElement, 
                                   int                  versionNumber);

    /** Serialize this object into the XML node that represents it.   
    @param      parent 
        Parent XML node of this object. Sending in a parent node allows an XML 
        node to be generated for this object if it doesn't already have one. If 
        no parent node is supplied and this object doesn't already have an XML 
        node, this object will become the root node for a new XML document. If 
        this object already has an XML node associated with it, no new nodes 
        are ever generated and the parent node is not used. 
    **/
    virtual void updateXMLNode(SimTK::Xml::Element& parent) const;

    /** Inlined means an in-memory Object that is not associated with
    an XMLDocument. **/
    bool getInlined() const;
    /** Mark this as inlined or not and optionally provide a file name
    to associate with the new XMLDocument for the non-inline case. If 
    there was already a document associated with this object it is
    deleted. **/
    void setInlined(bool aInlined, const std::string &aFileName="");

protected:
    /** When an object is initialized using the current values of its
    properties, it can set a flag indicating that it is up to date. This
    flag is automatically cleared when any property is modified. This allows
    objects to avoid expensive reinitialization if it is unnecessary (that is,
    whenever this %Object hands out writable access to a property). Note
    that use of this flag is entirely optional; most %Object classes don't
    have any expensive initialization to worry about.

    This flag is cleared automatically but if you want to clear it manually
    for testing or debugging, see clearObjectIsUpToDateWithProperties(). **/
    void setObjectIsUpToDateWithProperties() {
        _objectIsUpToDate = true;
    }

    /** For testing or debugging purposes, manually clear the "object is up to
    date with respect to properties" flag. This is normally done automatically
    when a property is modified. Setting the flag is always done manually,
    however, see setObjectIsUpToDateWithProperties(). **/
    void clearObjectIsUpToDateWithProperties() {
        _objectIsUpToDate = false;
    }

    /** Use this method only if you're deserializing from a file and the object
    is at the top level; that is, primarily in constructors that take a file
    name as input. **/
    void updateFromXMLDocument();
    /** Unconditionally set the XMLDocument associated with this object.
    Use carefully -- if there was already a document its heap space is
    lost here. **/
    void setDocument(XMLDocument* doc) {_document=doc;}

    /** Get a const pointer to the document (if any) associated with this
    object. **/
    const XMLDocument* getDocument() const {return _document;}
    /** Get a writable pointer to the document (if any) associated with this
    object. **/
    XMLDocument* updDocument() {return _document;}
public:
    /** If there is a document associated with this object then return the
    file name maintained by the document. Otherwise return an empty string. **/
    std::string getDocumentFileName() const;
    void setAllPropertiesUseDefault(bool aUseDefault);

    /** Write this %Object into an XML file of the given name; conventionally
    the suffix to use is ".osim". This is useful for writing out a Model that
    has been created programmatically, and also very useful for testing and
    debugging. **/
    bool print(const std::string& fileName) const;

    /** dump the XML representation of this %Object into an std::string and return it.
    Mainly intended for debugging and for use by the XML browser in the GUI. **/
    std::string dump(bool dumpName=false); 
    /**@}**/
    //--------------------------------------------------------------------------
    // ADVANCED/OBSCURE/QUESTIONABLE/BUGGY
    //--------------------------------------------------------------------------
    /** @name                      Advanced/Obscure
    Methods in this section are for specialized purposes not of interest to
    most OpenSim API users. For example, some of these are services needed by
    the OpenSim GUI which is written in Java. **/
    /**@{**/
    /** Return the name of this class as a string; i.e., "Object". See 
    getConcreteClassName() if you want the class name of the underlying concrete 
    object instead. Note that this method is automatically supplied for 
    every class declaration that derives from Object via the standard macro
    provided for that purpose. See introductory text for this Object class
    for more information. **/
    static const std::string& getClassName() 
    {   static std::string name ("Object"); return name; }

    /** Static function to control whether all registered objects and
    their properties are written to the defaults section of output files rather
    than only those values for which the default was explicitly overwritten
    when read in from an input file or set programmatically. **/
    static void setSerializeAllDefaults(bool shouldSerializeDefaults)
    {
        _serializeAllDefaults = shouldSerializeDefaults;
    }
    /** Report the value of the "serialize all defaults" flag. **/
    static bool getSerializeAllDefaults()
    {
        return _serializeAllDefaults;
    }

    /** Returns true if the passed-in string is "Object"; each %Object-derived
    class defines a method of this name for its own class name. **/
    static bool isKindOf(const char *type) 
    { 
        return (strcmp("Object",type)==0);
    } 
    /** The default implementation returns true only if the supplied string
    is "Object"; each %Object-derived class overrides this to match its own
    class name. **/
    virtual bool isA(const char *type) const
    { 
        return this->isKindOf(type); 
    } 

    /** %Set the debug level to get verbose output. Zero means no debugging. **/
    static void setDebugLevel(int newLevel) {
        _debugLevel=newLevel; 
    };
    /** Get current setting of debug level. **/
    static int getDebugLevel() {
        return _debugLevel; 
    };

    /** Wrapper to be used on Java side to display objects in tree; this returns
    just the object's name. **/
    const std::string& toString() const;

    #ifndef SWIG
    /** OBSOLETE: Get a reference to the PropertySet maintained by the 
    Object. **/
    PropertySet& getPropertySet() { return _propertySet; }
    const PropertySet& getPropertySet() const { return _propertySet; }
    #endif

    /** Use the clone() method to duplicate the given object unless the pointer
    is null in which case null is returned. **/
    static Object* SafeCopy(const Object *aObject) 
    {   return aObject ? aObject->clone() : 0; }

    /** OBSOLETE alternate name for registerType(). **/
    static void RegisterType(const Object& defaultObject) 
    {   registerType(defaultObject); }
    /** OBSOLETE alternate name for renameType(). **/
    static void RenameType(const std::string& oldName, 
                           const std::string& newName) 
    {   renameType(oldName, newName); }
    /**@}**/
    //--------------------------------------------------------------------------

//------------------------------------------------------------------------------
// PROTECTED METHODS
//------------------------------------------------------------------------------
protected:
    /** The default constructor is only for use by constructors of 
    derived types. Initializes all base class data members to innocuous 
    values. **/
    Object();

    /** Constructor from a file, to be called from other constructors that 
    take a file as input. **/
    explicit Object(const std::string& fileName, 
           bool aUpdateFromXMLNode = true) SWIG_DECLARE_EXCEPTION;

    /** Copy constructor is invoked automatically by derived classes with
    default copy constructors; otherwise it must be invoked explicitly. **/
    Object(const Object& source);

    /** Construct the base class portion of an %Object from a given Xml 
    element that describes this Object. Assumes latest XML file format; there
    is no provision for version numbering. **/
    explicit Object(SimTK::Xml::Element& aElement);


    /** Define a new single-value property of known type T, with the given 
    \a name, associated \a comment, and initial \a value. The name must be
    unique within this %Object's property table. 
    
    If T is an object type (i.e., derived from %Object), it is permissible for
    the property to be unnamed; pass an empty string for \a name. You will then
    be able to select the property using the object class name (that is,
    T::getClassName()) as though it were the property's name. An %Object can 
    thus only have one unnamed property of any particular object type.
    
    @returns Reference to the new Property object stored in this object's
             property table. 
    @see addOptionalProperty(), addListProperty() **/
    template <class T> PropertyIndex 
    addProperty(const std::string& name, 
                const std::string& comment, 
                const T&           value);

    /** Add an optional property, meaning it can contain either no value or
    a single value. Here no initial value is provided. The
    property must have a name (the empty string is not acceptable), and that
    name must be unique within this %Object's property table.
    @returns Reference to the new Property object stored in this object's
             property table. 
    @see addProperty(), addListProperty() **/
    template <class T> PropertyIndex 
    addOptionalProperty(const std::string& name,
                        const std::string& comment);

    /** Add an optional property, meaning it can contain either no value or
    a single value. Here an initial value is provided. The
    property must have a name (the empty string is not acceptable), and that
    name must be unique within this %Object's property table.
    @returns Reference to the new Property object stored in this object's
             property table. 
    @see addProperty(), addListProperty() **/
    template <class T> PropertyIndex
    addOptionalProperty(const std::string& name,
                        const std::string& comment,
                        const T& value);

    /** Define a new list-valued property of known type T, with the given 
    \a name, associated \a comment, minimum (==0) and maximum (>0) allowable
    list lengths, and a zero-length initial value. The
    property must have a name (the empty string is not acceptable), and that
    name must be unique within this %Object's property table.
    @returns The PropertyIndex of this property in the property table for this
             object. 
    @see addProperty(), addOptionalProperty() **/
    template <class T> PropertyIndex
    addListProperty(const std::string& name, 
                    const std::string& comment,
                    int minSize, int maxSize);

    /** Define a new list-valued property as above, but assigning an initial
    value via some templatized container class that supports size() and 
    indexing. Here the minimum size may be greater than zero, provided that
    the initial value has at least that many element (and no more than the
    allowed maximum).
    @returns The PropertyIndex of this property in the property table for this
             object. 
    @see addProperty(), addOptionalProperty() **/
    template <class T, template<class> class Container> PropertyIndex 
    addListProperty(const std::string&  name, 
                    const std::string&  comment,
                    int minSize, int maxSize,
                    const Container<T>& valueList);

    /** Look up a property by name and return its PropertyIndex if it is
    found. If no property of that name is present, the returned index
    will be invalid; check with isValid(). **/
    // Note: only works for new properties.
    PropertyIndex getPropertyIndex(const std::string& name) const
    {   const int ix = _propertyTable.findPropertyIndex(name); 
        if (ix >= 0) return PropertyIndex(ix);
        return PropertyIndex();
    }

    /** Look up an unnamed property by the type of object it contains,
    and return its PropertyIndex if it is found. If no unnamed property of that
    type is present, the returned index will be invalid; check with 
    isValid(). **/
    // Note: only works for new properties.
    template <class T> PropertyIndex getPropertyIndex() const
    {   const int ix = _propertyTable.findPropertyIndex(T::getClassName()); 
        if (ix >= 0) return PropertyIndex(ix);
        return PropertyIndex();
    }

//--------------------------------------------------------------------------
// PRIVATE METHODS
//--------------------------------------------------------------------------
private:
    void setNull();

    // Functions to support deserialization. 
    void generateXMLDocument();

    void updateDefaultObjectsFromXMLNode();
    void updateDefaultObjectsXMLNode(SimTK::Xml::Element& aParent);


//==============================================================================
// DATA
//==============================================================================
public:
    #ifndef SWIG
    /** Name used for default objects when they are serialized. */
    static const std::string DEFAULT_NAME;
    #endif

protected:
    /** OBSOLETE: Property_Deprecated set for serializable member variables of 
    this and derived classes. */
    PropertySet _propertySet;

private:
    // Array holding a default value for each of the registered object types. 
    // Each object type only appears once in this array. Renamed types usually
    // do not have separate registered objects; they are just used to locate 
    // one of the current ones.
    static ArrayPtrs<Object>                    _registeredTypes;

    // Map from concrete object class name string to a default object of that 
    // type kept in the above array of registered types. Renamed types are *not* 
    // normally entered here; the names are mapped separately using the map 
    // below.
    static std::map<std::string,Object*>        _mapTypesToDefaultObjects;

    // Map types that have been renamed to their new names, which can
    // then be used to find them in the default object map. This lets us 
    // recognize the old names while converting to the new ones internally
    // so that they will be updated when written out. It also allows one 
    // to map one registered type to a different one programmatically, because
    // we'll look up the name in the rename table first prior to searching
    // the registered types list.
    static std::map<std::string,std::string>    _renamedTypesMap;

    // Global flag to indicate if all registered objects are to be written in 
    // a "defaults" section.
    static bool _serializeAllDefaults;

    // Debug level: 
    //  0: Hides non fatal warnings 
    //  1: Shows illegal tags 
    //  2: level 1 + registration troubleshooting
    //  3: 2 + more verbose troubleshooting of Object (de)serialization. When 
    //     used from Java wrapping in GUI/Matlab this catches all exceptions 
    //     thrown by the low-level libraries which is slower but helpful in 
    //     troubleshooting.
    static int      _debugLevel;

    // The name of this object.
    std::string     _name;
    // A short description of the object.
    std::string     _description;

    // List of authors who contributed to the implementation of concrete object.
    std::string     _authors;
    // List of references that should be cited when using this concrete object.
    std::string     _references;

    // Property table for serializable properties of this and derived classes.
    PropertyTable   _propertyTable;
    // This flag is cleared automatically whenever a property is changed. It 
    // is initialized to false and is only set manually.
    bool            _objectIsUpToDate;

    // The XML document, if any, associated with this object.
    // This is mutable since it's cached on deserialization and is 
    // kept up to date to maintain "defaults" and document file path
    //TODO: why does an Object need to know where it was last written? Seems flaky and should be revisited
    mutable XMLDocument     *_document;
    // Flag indicating whether the object is serialized to this _document or 
    // to another fresh document, also cached for subsequent printing/writing.
    mutable bool            _inlined;

//==============================================================================
};  // END of class Object



//==============================================================================
//                   OBJECT TEMPLATE METHOD IMPLEMENTATION
//==============================================================================
// This only works for the new properties -- it won't see deprecated ones.
template <class T> bool Object::
hasProperty() const {
    // Look it up by T's object class name if that's allowed.
    if (Property<T>::TypeHelper::IsObjectType) {
        return _propertyTable.hasProperty
            (Property<T>::TypeHelper::getTypeName());
    }

    throw OpenSim::Exception
       ("hasProperty<T>(): nameless property lookup by object class name "
        "only allowed when T is an Object-derived type, but T=" 
        + std::string(SimTK::NiceTypeName<T>::name()) + ". For lookup by "
        "property name instead, use hasProperty(\"prop_name\").");
    return false;
}

template <class T> const Property<T>& Object::
getProperty(const PropertyIndex& index) const {
    return _propertyTable.getProperty<T>(index);
}

template <class T> Property<T>& Object::
updProperty(const PropertyIndex& index) {
    _objectIsUpToDate = false; // property may be changed
    return _propertyTable.updProperty<T>(index);
}

template <class T> PropertyIndex Object::
addProperty(const std::string& name, 
            const std::string& comment,
            const T&           value)
{
    // Restrict to exactly one value. If there is no name, this will throw
    // an exception if T is a simple (non-object) type.
    Property<T>* p = Property<T>::TypeHelper::create(name, true);

    p->setComment(comment);
    p->appendValue(value);
    p->setValueIsDefault(true);

    // Note that an unnamed, one-object property will use the object class name
    // as a name for lookup purposes.
    return PropertyIndex(_propertyTable.adoptProperty(p));
}

template <class T> PropertyIndex Object::
addOptionalProperty(const std::string& name, 
                    const std::string& comment, 
                    const T&    value)
{
    if (name.empty())
        throw OpenSim::Exception(
            "Object::addOptionalProperty(): an optional property must have "
            "a name. (Object " + getName() + ").");

    Property<T>* p = Property<T>::TypeHelper::create(name, false);
    p->setAllowableListSize(0,1);
    p->setComment(comment);
    p->appendValue(value);
    p->setValueIsDefault(true);

    return PropertyIndex(_propertyTable.adoptProperty(p));
}

template <class T> PropertyIndex Object::
addOptionalProperty(const std::string& name, 
                    const std::string& comment)
{
    if (name.empty())
        throw OpenSim::Exception(
            "Object::addOptionalProperty(): an optional property must have "
            "a name. (Object " + getName() + ").");

    Property<T>* p = Property<T>::TypeHelper::create(name, false);
    p->setAllowableListSize(0,1);
    p->setComment(comment);
    p->setValueIsDefault(true);

    return PropertyIndex(_propertyTable.adoptProperty(p));
}

template <class T> PropertyIndex Object::
addListProperty(const std::string& name, 
                const std::string& comment,
                int minSize, int maxSize)
{
    if (name.empty())
        throw OpenSim::Exception(
            "Object::addListProperty(): a list property must have a name. "
            "(Object " + getName() + ").");

    if (minSize > 0)
        throw OpenSim::Exception(
            "Object::addListProperty(): list property " + name 
            + " has a minimum list size of " + SimTK::String(minSize)
            + " so must be given an initial value of at least that size "
              "(Object " + getName() + ").");


    Property<T>* p = Property<T>::TypeHelper::create(name, false);
    p->setAllowableListSize(minSize, maxSize);
    p->setComment(comment);
    p->setValueIsDefault(true);

    return PropertyIndex(_propertyTable.adoptProperty(p));
}

template <class T, template<class> class Container> PropertyIndex Object::
addListProperty(const std::string&  name, 
                const std::string&  comment,
                int minSize, int maxSize,
                const Container<T>& valueList)
{
    if (name.empty())
        throw OpenSim::Exception(
            "Object::addListProperty(): a list property must have a name. "
            "(Object " + getName() + ").");

    if (valueList.size() < minSize || valueList.size() > maxSize)
        throw OpenSim::Exception(
            "Object::addListProperty(): list property " + name 
            + " has allowable list size " + SimTK::String(minSize) + ".."
            + SimTK::String(maxSize) + " but initial value had size "
            + SimTK::String(valueList.size()) + ".");

    Property<T>* p = Property<T>::TypeHelper::create(name, false);
    p->setAllowableListSize(minSize, maxSize);
    p->setComment(comment);
    for (int i=0; i < (int)valueList.size(); ++i)
        p->appendValue(valueList[i]);
    p->setValueIsDefault(true);

    return PropertyIndex(_propertyTable.adoptProperty(p));
}



//==============================================================================
//                  DERIVED OBJECT BOILERPLATE MACROS
//==============================================================================

/** @name               Object Declaration Macros
One of these macros must appear as the first line of any class declaration
that derives directly or indirectly from %OpenSim's Object class. In almost
all cases, the right macro to use is \c OpenSim_DECLARE_CONCRETE_OBJECT().

Use of these macros provides:
  - a public typedef Super that is the immediate parent class,
  - implementation of required Object pure virtual methods, including
    the clone() method that will create a new heap-allocated copy of any 
    concrete Object,
  - uniform treatment of class names, which are used as tags in XML and for 
    interfacing with Java using class names as strings to identify C++ 
    objects. The static getClassName() returns the name of any class, and
    the member getConcreteClassName() returns the class name of the concrete 
    object being referenced, and
  - an assortment of methods used only for interfacing with Java.
**/
/**@{**/
/** Macro to be included as the first line of the class declaration for
any non-templatized, concrete class that derives from OpenSim::Object. You 
should use this for any such class, even if you intend to derive more specific 
concrete objects from it. Don't use this for a still-abstract class, or a 
templatized concrete class like Set\<T>.
@relates OpenSim::Object **/
#define OpenSim_DECLARE_CONCRETE_OBJECT(ConcreteClass, SuperClass)             \
OpenSim_OBJECT_ANY_DEFS(ConcreteClass, SuperClass);                            \
OpenSim_OBJECT_NONTEMPLATE_DEFS(ConcreteClass, SuperClass);                    \
OpenSim_OBJECT_CONCRETE_DEFS(ConcreteClass);

/** Macro to be included as the first line of the class declaration for
any still-abstract class that derives from OpenSim::Object. These are classes
that represent categories of objects, like Function and ModelComponent. This
macro leaves Object pure virtuals clone() and getConcreteClassName() unimplemented,
however it does redeclare the return type of clone() to be ConcreteClass*.
@relates OpenSim::Object **/
#define OpenSim_DECLARE_ABSTRACT_OBJECT(ConcreteClass, SuperClass)             \
OpenSim_OBJECT_ANY_DEFS(ConcreteClass, SuperClass);                            \
OpenSim_OBJECT_NONTEMPLATE_DEFS(ConcreteClass, SuperClass);                    \
OpenSim_OBJECT_ABSTRACT_DEFS(ConcreteClass);

/** Macro to be included as the first line of the class declaration for
any templatized, concrete class that derives from OpenSim::Object, 
like Set\<T>.
@relates OpenSim::Object **/
#define OpenSim_DECLARE_CONCRETE_OBJECT_T(ConcreteClass, TArg, SuperClass)     \
OpenSim_OBJECT_ANY_DEFS(ConcreteClass, SuperClass);                            \
OpenSim_OBJECT_TEMPLATE_DEFS(ConcreteClass, TArg, SuperClass);                 \
OpenSim_OBJECT_CONCRETE_DEFS(ConcreteClass);

/** Macro to be included as the first line of the class declaration for
any templatized, still-abstract class that derives from OpenSim::Object.
@relates OpenSim::Object **/
#define OpenSim_DECLARE_ABSTRACT_OBJECT_T(ConcreteClass, TArg, SuperClass)     \
OpenSim_OBJECT_ANY_DEFS(ConcreteClass, SuperClass);                            \
OpenSim_OBJECT_TEMPLATE_DEFS(ConcreteClass, TArg, SuperClass);                 \
OpenSim_OBJECT_ABSTRACT_DEFS(ConcreteClass);
/**@}**/

// Hide helper macros from Doxygen -- they do not appear in code anywhere
// but right here. They are used to construct the macros that are used in
// various circumstances without duplicating any definitions.

// This class allows us to get the class name for template arguments using
// getClassName() when it is available, otherwise a specialization.
template <class T> struct Object_GetClassName 
{   static const std::string& name() {return T::getClassName();} };
template <> struct Object_GetClassName<bool> 
{   static const std::string name() {return "bool";} };
template <> struct Object_GetClassName<signed char> 
{   static const std::string name() {return "char";} };
template <> struct Object_GetClassName<unsigned char> 
{   static const std::string name() {return "char";} };
template <> struct Object_GetClassName<char> 
{   static const std::string name() {return "char";} };
template <> struct Object_GetClassName<short int> 
{   static const std::string name() {return "int";} };
template <> struct Object_GetClassName<unsigned short int> 
{   static const std::string name() {return "int";} };
template <> struct Object_GetClassName<int> 
{   static const std::string name() {return "int";} };
template <> struct Object_GetClassName<unsigned int> 
{   static const std::string name() {return "int";} };
template <> struct Object_GetClassName<long int> 
{   static const std::string name() {return "int";} };
template <> struct Object_GetClassName<unsigned long int> 
{   static const std::string name() {return "int";} };
template <> struct Object_GetClassName<long long int> 
{   static const std::string name() {return "int";} };
template <> struct Object_GetClassName<unsigned long long int> 
{   static const std::string name() {return "int";} };
template <> struct Object_GetClassName<float> 
{   static const std::string name() {return "float";} };
template <> struct Object_GetClassName<double> 
{   static const std::string name() {return "double";} };
template <> struct Object_GetClassName<long double> 
{   static const std::string name() {return "double";} };
template <> struct Object_GetClassName<std::string> 
{   static const std::string name() {return "string";} };
template <> struct Object_GetClassName<SimTK::Vec2> 
{   static const std::string name() {return "Vec2";} };
template <> struct Object_GetClassName<SimTK::Vec3> 
{   static const std::string name() {return "Vec3";} };
template <> struct Object_GetClassName<SimTK::Vec6> 
{   static const std::string name() {return "Vec6";} };
template <> struct Object_GetClassName<SimTK::Vector_<SimTK::Real>>
{   static const std::string name() {return "Vector"; } };
template <> struct Object_GetClassName<SimTK::Vector_<SimTK::Vec3>>
{   static const std::string name() {return "Vector_<Vec3>";} };
template <> struct Object_GetClassName<SimTK::Vector_<SimTK::Vec6>>
{   static const std::string name() {return "Vector_<Vec6>";} };
template <> struct Object_GetClassName<SimTK::Vector_<SimTK::SpatialVec>>
{   static const std::string name() {return "Vector_<SpatialVec>";} };
template <> struct Object_GetClassName<SimTK::SpatialVec>
{   static const std::string name() {return "SpatialVec";} };
template <> struct Object_GetClassName<SimTK::Transform>
{   static const std::string name() {return "Transform";} };

#define OpenSim_OBJECT_ANY_DEFS(ConcreteClass, SuperClass)                     \
public:                                                                        \
/** @cond developer **/                                                        \
/** This typedef might be useful within the member functions of this class. */ \
/** \internal This is generated by the `OpenSim_DECLARE_*_OBJECT` macros.   */ \
typedef ConcreteClass Self;                                                    \
/** Use this typedef to refer to the superclass of this class.              */ \
/** Avoid using the explicit type name of the superclass; this would        */ \
/** introduce bugs if the superclass is changed.                            */ \
/** \internal This is generated by the `OpenSim_DECLARE_*_OBJECT` macros.   */ \
typedef SuperClass    Super;                                                   \
/** @endcond **/                                                               \
OpenSim_OBJECT_JAVA_DEFS(ConcreteClass);

// For non-template classes, the class name is identical to the supplied
// ConcreteClass argument.
#define OpenSim_OBJECT_NONTEMPLATE_DEFS(ConcreteClass, SuperClass)             \
/** @name Auto-generated functions                                          */ \
/** @{                                                                      */ \
/** This returns "##ConcreteClass##"                                        */ \
/** See getConcreteClassName() if you want the class name of the underlying */ \
/** concrete object instead.                                                */ \
/** \internal This is generated by the `OpenSim_DECLARE_*_OBJECT` macros.   */ \
static const std::string& getClassName()                                       \
{   static std::string name(#ConcreteClass); return name; }                    \
/** @}*/

// For template classes ConcreteClass<TemplateArg>, we construct the class
// name by assembling the pieces.
#define OpenSim_OBJECT_TEMPLATE_DEFS(ConcreteClass, TArg, SuperClass)          \
/** @name Auto-generated functions                                          */ \
/** @{                                                                      */ \
/** This returns "##ConcreteClass##_<T>_".                                  */ \
/** T is the template argument for this class.                              */ \
/** See getConcreteClassName() if you want the class name of the underlying */ \
/** concrete object instead.                                                */ \
/** \internal This is generated by the `OpenSim_DECLARE_*_OBJECT` macros.   */ \
static const std::string& getClassName()                                       \
{   static std::string name = #ConcreteClass "_"                               \
                              + Object_GetClassName<TArg>::name()              \
                              + "_";                                           \
    return name; }                                                             \
/** @}*/

// This provides definitions for the two Object pure virtuals clone() and
// getConcreteClassName().
#define OpenSim_OBJECT_CONCRETE_DEFS(ConcreteClass)                            \
/** @name Auto-generated functions                                          */ \
/** @{                                                                      */ \
ConcreteClass* clone() const override {return new ConcreteClass(*this);}       \
const std::string& getConcreteClassName() const override                       \
{   return getClassName(); }                                                   \
/** @}*/                                                                       \
private:

// This leaves the two Object pure virtuals clone() and getConcreteClassName()
// unimplemented, but changes the return type of clone() to ConcreteClass*,
// which allows it to be invoked ConcreteClass::clone() and return the correct
// pointer type.
#define OpenSim_OBJECT_ABSTRACT_DEFS(ConcreteClass)                            \
/** @name Auto-generated functions                                          */ \
/** @{                                                                      */ \
ConcreteClass* clone() const override = 0;                                     \
const std::string& getConcreteClassName() const override = 0;                  \
/** @}*/                                                                       \
private:

// Add public static method declaration in class to assist in downcasting
// arbitrary objects to the new type to support dynamic casting across JNI.
#define OpenSim_OBJECT_JAVA_DEFS(thisClass) \
  public: \
  /** @name Auto-generated functions */ \
  /** @{ */ \
  /** For use in MATLAB and Python to access the concrete class. */ \
  /** Example: `cObj = %##thisClass##.safeDownCast(obj)`. */ \
  /** This is equivalent to `dynamic_cast<##thisClass##*>(obj)` in C++. */ \
  static thisClass* safeDownCast(OpenSim::Object *obj) \
  { \
      return dynamic_cast<thisClass *>(obj); \
  } \
  /** @cond developer */ \
  /** This allows copy assignment in the Java GUI. */ \
  /** @throws Exception if the argument is not of type thisClass##. */ \
  void assign(Object &aObject) override \
  { \
      if (safeDownCast(&aObject)!=0) { \
          *this = *((thisClass*)(&aObject)); \
     } else { \
      throw OpenSim::Exception(std::string(#thisClass)+ \
              "::copy() called with object (name = " + aObject.getName() \
              + ", type = " + aObject.getConcreteClassName()+").", \
              __FILE__,__LINE__); \
     } \
  } \
  /** @endcond */ \
  /** @}*/

//==============================================================================
//                        OBJECT PROPERTY IMPLEMENTATION
//==============================================================================
// These methods of ObjectProperty are defined here because they depend on 
// methods of Object. See Property.h for ObjectProperty's declaration.
/** @cond **/ // Not for Doxygen.
template <class T> inline std::string 
ObjectProperty<T>::toString() const {
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

template <class T> inline bool 
ObjectProperty<T>::isAcceptableObjectTag
   (const std::string& objectTypeTag) const { 
    return Object::isObjectTypeDerivedFrom<T>(objectTypeTag); 
}

template <class T> inline bool 
ObjectProperty<T>::isEqualTo(const AbstractProperty& other) const {
    // Check here rather than in base class because the old
    // Property_Deprecated implementation can't copy this flag right.
    if (this->getValueIsDefault() != other.getValueIsDefault())
        return false;
    assert(this->size() == other.size()); // base class checked
    const ObjectProperty& otherO = ObjectProperty::getAs(other);
    for (int i=0; i<objects.size(); ++i) {
        const T* const thisp  = objects[i].get();
        const T* const otherp = otherO.objects[i].get();
        if (thisp == otherp)
            continue; // same object or both null
        if (!(thisp && otherp))
            return false; // only one is null; they are different
        if (!(*thisp == *otherp)) // delegate to object's operator==()
            return false;
    }
    return true;
}


// Property element is a compound element, consisting of subelements
// each of which is one of the object values.
template <class T> inline void 
ObjectProperty<T>::readFromXMLElement
    (SimTK::Xml::Element& propertyElement,
    int                  versionNumber)
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

        const Object* registeredObj = 
            Object::getDefaultInstanceOfType(objTypeTag);

        if (!registeredObj) {
            std::cerr 
                << "Encountered unrecognized Object typename " 
                << objTypeTag << " while reading property " << this->getName()
                << ". There is no registered Object of this type; ignoring.\n";
            continue;                        
        }

        // Check that the object type found is derived from T.
        if (!dynamic_cast<const T*>(registeredObj)) {
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
        object->readObjectFromXMLNodeOrFile(*iter, versionNumber);

        T* objectT = dynamic_cast<T*>(object);
        assert(objectT); // should have worked by construction
        adoptAndAppendValueVirtual(objectT); // don't copy
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
template <class T> inline void 
ObjectProperty<T>::writeToXMLElement
    (SimTK::Xml::Element& propertyElement) const 
{
    for (int i=0; i < objects.size(); ++i)
        (objects[i])->updateXMLNode(propertyElement);
}


template <class T> inline void 
ObjectProperty<T>::setValueAsObject(const Object& obj, int index) {
    if (index < 0 && this->getMaxListSize()==1)
        index = 0;
    T* newObjT = dynamic_cast<T*>(obj.clone());
    if (newObjT == NULL) 
        throw OpenSim::Exception
            ("ObjectProperty<T>::setValueAsObject(): the supplied object"
            + obj.getName() + " was of type " + obj.getConcreteClassName()
            + " which can't be stored in this " + objectClassName
            + " property " + this->getName());

    objects[index] = newObjT;
}
/** @endcond **/

//==============================================================================
//                    ABSTRACT PROPERTY TEMPLATE METHODS
//==============================================================================
// TODO: these are defined here in order to provide support for the old
// deprecated property system under the AbstractProperty umbrella. Move to
// Property.h when the deprecated code is removed.

template <class T> inline const T& AbstractProperty::
getValue(int index) const {
    //TODO: temporary support for obsolete properties
    const Property_Deprecated* pd = 
        dynamic_cast<const Property_Deprecated*>(this);
    if (pd) {
        if (pd->isArrayProperty()) {
            return pd->getValueArray<T>()[index];
        } else {
            return pd->getValue<T>();
        }
    }

    const Property<T>* p = dynamic_cast<const Property<T>*>(this);
    if (p == NULL)
        throw Exception("AbstractProperty::getValue(): property "
                        + getName() + " is not of type " 
                        + std::string(SimTK::NiceTypeName<T>::name()));
    return p->getValue(index);
}

template <class T> inline T& AbstractProperty::
updValue(int index) {
    setValueIsDefault(false); // assume it is being changed
    //TODO: temporary support for obsolete properties
    Property_Deprecated* pd = dynamic_cast<Property_Deprecated*>(this);
    if (pd) {
        return pd->isArrayProperty()
            ? pd->getValueArray<T>()[index]
            : pd->getValue<T>();
    }

    Property<T>* p = dynamic_cast<Property<T>*>(this);
    if (p == NULL)
        throw Exception("AbstractProperty::updValue(): property "
                        + getName() + " is not of type " 
                        + std::string(SimTK::NiceTypeName<T>::name()));
    return p->updValue(index);
}

template <class T> inline int AbstractProperty::
appendValue(const T& value) {
    setValueIsDefault(false);
    //TODO: temporary support for obsolete properties
    Property_Deprecated* pd = dynamic_cast<Property_Deprecated*>(this);
    if (pd) {
        if (!pd->isArrayProperty())
            throw Exception
               ("AbstractProperty::appendValue(): deprecated property "
                + getName() + " is not an Array property; can't append.");
        pd->getValueArray<T>().append(value);
        return pd->getNumValues()-1;
    }

    Property<T>* p = dynamic_cast<Property<T>*>(this);
    if (p == NULL)
        throw Exception("AbstractProperty::appendValue(): property "
                        + getName() + " is not of type " 
                        + std::string(SimTK::NiceTypeName<T>::name()));
    return p->appendValue(value);
}



}; //namespace

#endif // OPENSIM_OBJECT_H_
