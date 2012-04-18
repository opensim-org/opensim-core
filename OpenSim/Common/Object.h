#ifndef OPENSIM_OBJECT_H_
#define OPENSIM_OBJECT_H_
// Object.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c) 2005-12, Stanford University. All rights reserved. 
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

#ifdef WIN32
#pragma warning( disable : 4251 )
#pragma warning( disable : 4786 )
#pragma warning( disable : 4660 )
#endif

// INCLUDES

#include "osimCommonDLL.h"
#include "XMLDocument.h"
#include "PropertySet.h"
#include "PropertyTable.h"
#include "Property.h"

#include <cstring>
#include <cassert>
#include <map>

// DISABLES MULTIPLE INSTANTIATION WARNINGS


// EXPORT LINE FOR MICROSOFT VISUAL C++
#ifdef WIN32
#ifndef SWIG
template class OSIMCOMMON_API OpenSim::ArrayPtrs<OpenSim::Object>;
#endif
#endif


#ifdef SWIG
	#ifdef OSIMCOMMON_API
		#undef OSIMCOMMON_API
	#endif
	#define OSIMCOMMON_API
	#ifdef OVERRIDE_11
		#undef OVERRIDE_11
	#endif
	#define OVERRIDE_11
	#ifdef FINAL_11
		#undef FINAL_11
	#endif
	#define FINAL_11
	#define SWIG_DECLARE_EXCEPTION throw(OpenSim::Exception)
#else
	#define SWIG_DECLARE_EXCEPTION
#endif


namespace OpenSim { 

// CONSTANTS
const char ObjectDEFAULT_NAME[] = "default";

class VisibleObject;
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
tree structure that is the representation of an %OpenSim Model. 

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
that will be commonly used in one place in the XML file rather then with each 
object which leads to smaller files that are easier to read. Property values
that obtain their values from the defaults and are not subsequently overridden
are marked as being default values, allowing us to avoid writing
them back out when serializing.

@author Frank C. Anderson, Ayman Habib, Ajay Seth, Michael Sherman 
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

	/**
	 * Methods to support making the object displayable in the GUI or Visualizer
	 * Implemented only in few objects
	 */
	/** Get const pointer to VisibleObject that contains geometry */
	virtual const VisibleObject *getDisplayer() const { return 0; };
	/** get Non const pointer to VisibleObject */
	virtual VisibleObject *updDisplayer() { return 0; };


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
    /** Copy assignment copies only the simple base class fields, not the 
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
	/** Set the name of the Object. */
	void setName(const std::string& name);
	/** Get the name of this Object. */
	const std::string& getName() const;
	/** Set description, a one-liner summary. */
	void setDescription(const std::string& description);
	/** Get description, a one-liner summary. */
	const std::string& getDescription() const;

	/** Get Authors of this Object */
	const std::string& getAuthors() const { return _authors; };
	/** Set Authors of this object, call this method in your constructor if needed */
	void setAuthors(const std::string& authors) { _authors=authors; };

	/** Get references or publications to cite if using this object. */
	const std::string& getReferences() const { return _references; };
	/** Set references or publications to cite if using this object. */
	void setReferences(const std::string& references) 
    {   _references=references; };


	//--------------------------------------------------------------------------
	// PUBLIC ACCESS TO PROPERTIES
	//--------------------------------------------------------------------------
    /** @name              Public access to properties
    Methods in this section are for public access to the properties maintained
    by this OpenSim %Object. Methods for dealing with properties using their
    base class AbstractProperty support various type-independent property 
    services. That is particularly useful for %Object-containing properties
    since the objects can be obtained without knowing their concrete types.
    For simple types (e.g. int, std::string) you can only obtain the values if
    you know the expected type. For simple types, or when you know the 
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
    an AbstractProperty.  An exception is thrown if no property by this name
    is present in this %Object. **/
    AbstractProperty& updPropertyByName(const std::string& name);

    /** Return true if this %Object contains an unnamed, one-object property
    that contains objects of the given template type T. The type must match
    exactly the type used when this property was created with 
    addProperty<T>(). **/
    template <class T> bool hasProperty() const;

	/** Get property \a name of known type Property\<T> as a const reference; 
    the property must exist. If this is a nameless, one-object property then 
    the name can be empty (the default if you leave it off altogether) and the
    property will be looked up by the object class name of template type T 
    instead. **/
	template <class T> const Property<T>& 
    getProperty(const std::string& name = "") const;

	/** Get property \a name of known type Property\<T> as a writable reference;
    the property must exist.  If this is a nameless, one-object property then the 
    name can be empty and the property will be looked up by the object class 
    name of template type T instead. **/
	template <class T> Property<T>& 
    updProperty(const std::string& name = "");

    /** Obtain a const reference to the value of known type T that is stored
    in the property named \a name, which must exist. **/
	template <class T> const T& 
    getPropertyValue(const std::string& name = "") const;

    /** Obtain a writable reference to the value of known type T that is stored
    in the property named \a name, which must exist. **/
	template <class T> T& 
    updPropertyValue(const std::string& name = "");

    /** Assign a new value list to a list- or single-valued Property, using an 
    arbitrary Container\<T> as long as the container supports a size() method 
    and indexing with operator[]. An exception is thrown if the number of
    values in \a valueList is inappropriate for this property. **/
    template <class T, template<class> class Container> void
    setPropertyValue(const std::string& name, 
                     const Container<T>& valueList);
    /** Abbreviation permitted for unnamed property. **/
    template <class T, template<class> class Container> void
    setPropertyValue(const Container<T>& valueList)
    {   return setPropertyValue("", valueList); }

    /** Assign a new value list to a list- or single-valued Property by copying
    the value list of the given \c source Property, along with the flag 
    indicating whether the value is just the default. An exception is thrown if 
    the number of values in \a source is inappropriate for this property. **/
    template <class T> void
    setPropertyValue(const std::string& name, 
                     const Property<T>& source);
    /** Abbreviation permitted for unnamed property. **/
    template <class T> void
    setPropertyValue(const Property<T>& source)
    {   return setPropertyValue("", source); }

    /** Set the value of the property named \a name, which must exist and be
    a property of type T. **/
	template <class T> void 
    setPropertyValue(const std::string& name, const T& value);
    /** Abbreviation permitted for unnamed property. **/
	template <class T> void 
    setPropertyValue(const T& value)
    {   setPropertyValue("", value); }


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
    **/
	static void PrintPropertyInfo(std::ostream&      os,
					              const std::string& classNameDotPropertyName);
    /** Same as the other signature but the class name and property name are
    provided as two separate strings. **/
	static void PrintPropertyInfo(std::ostream&         os,
					              const std::string&    className,
                                  const std::string&    propertyName);
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
    their modern names using the renameType() method. **/
    /**@{**/

    /** Register an instance of a class; if the class is already registered it
    will be replaced. This is normally called as part of the static
    intialization of a dynamic library (DLL). The supplied object's concrete
    class name will be used as a key, and a \e copy (via clone()) of the 
    supplied %Object is used as the default value for objects of this type when 
    created (typically during the deserialization process when reading an 
    XML file). **/
	static void registerType(const Object& defaultObject);

	/** Support versioning by associating the current %Object type with an 
    old name. This is only allowed if \a newTypeName has already been 
    registered with registerType(). **/
	static void renameType(const std::string& oldTypeName, 
                           const std::string& newTypeName);

    /** Return a pointer to the default instance of the registered (concrete)
    %Object whose class name is given, or NULL if the type is not registered.
    Note that this refers to the default %Object instance that is stored with
    the %Object class; do not delete it! If you want a copy of this object
    instead, use newInstanceOfType(). The given \a concreteClassName will be
    mapped through the renamed type table if necessary but the returned object
    will always have the new type name, which may differ from the supplied 
    one. 
    @see registerType(), renameType() **/
    static const Object* 
    getDefaultInstanceOfType(const std::string& concreteClassName);

    /** Return true if the given concrete object type represents a subclass of 
    the template object type T, and thus could be referenced with a T*. The 
    object type to be tested is given by its class name as a string. 
    For this to work the name must represent an already-registered object
    type. If necessary \a concreteClassName will be mapped through the renamed
    type table, so we'll return true if the class it maps to satisfies the
    condition.
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
    instances are returned; renamed types will not appear. The result returned 
    in \a typeNames should not be cached while more shared libraries or plugins 
    are loaded, because more types may be registered as a result. Instead the 
    list should be reconstructed whenever in doubt. **/
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

	/** Use this method only if you're deserializing from a file and the object
    is at the top level; that is, primarily in constructors that take a file
    name as input. **/
	void updateFromXMLDocument();

	/** Use this method to deserialize an object from a SimTK::Xml::Element. The 
    element is assumed to be in the format consistent with the passed-in 
    \a versionNumber. **/
	virtual void updateFromXMLNode(SimTK::Xml::Element& node, 
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
	virtual void updateXMLNode(SimTK::Xml::Element& parent);

	/** Inlined means an in-memory Object that is not associated with
    an XMLDocument. **/
	bool getInlined() const;
    /** Mark this as inlined or not and optionally provide a file name
    to associate with the new XMLDocument for the non-inline case. If 
    there was already a document associated with this object it is
    deleted. **/
	void setInlined(bool aInlined, const std::string &aFileName="");
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
    /** If there is a document associated with this object then return the
    file name maintained by the document. Otherwise return an empty string. **/
	std::string getDocumentFileName() const;
	void setAllPropertiesUseDefault(bool aUseDefault);

	/** Write this %Object into an XML file of the given name; conventionally
    the suffix to use is ".osim". This is useful for writing out a Model that
    has been created programmatically, and also very useful for testing and
    debugging. **/
    bool print(const std::string& fileName);
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

    /** Set the debug level to get verbose output. Zero means no debugging. **/
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

	/** OBSOLETE: Get a reference to the PropertySet maintained by the 
    Object. **/
	PropertySet& getPropertySet() { return _propertySet; }
    #ifndef SWIG
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
	template <class T> Property<T>& 
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
    template <class T> Property<T>& 
    addOptionalProperty(const std::string& name,
                        const std::string& comment);

    /** Add an optional property, meaning it can contain either no value or
    a single value. Here an initial value is provided. The
    property must have a name (the empty string is not acceptable), and that
    name must be unique within this %Object's property table.
    @returns Reference to the new Property object stored in this object's
             property table. 
    @see addProperty(), addListProperty() **/
    template <class T> Property<T>& 
    addOptionalProperty(const std::string& name,
                        const std::string& comment,
                        const T& value);

    /** Define a new list-valued property of known type T, with the given 
    \a name, associated \a comment, and a zero-length initial value. The
    property must have a name (the empty string is not acceptable), and that
    name must be unique within this %Object's property table.
    @returns Reference to the new Property object stored in this object's
             property table. 
    @see addProperty(), addOptionalProperty() **/
	template <class T> Property<T>& 
    addListProperty(const std::string& name, 
                    const std::string& comment);

    /** Define a new list-valued property as above, but assigning an initial
    value via some templatized container class that supports size() and 
    indexing.
    @returns Reference to the new Property object stored in this object's
             property table. 
    @see addProperty(), addOptionalProperty() **/
    //TODO: this should only require that the container have a forward
    //iterator.
	template <class T, template<class> class Container> Property<T>& 
    addListProperty(const std::string&  name, 
                    const std::string&  comment,
                    const Container<T>& valueList);

//--------------------------------------------------------------------------
// PRIVATE METHODS
//--------------------------------------------------------------------------
private:
	void setNull();
	void setupProperties();
	void init();

	// Functions to support deserialization. 
	void generateXMLDocument();
	static void InitializeObjectFromXMLNode(Property_Deprecated *aProperty, const SimTK::Xml::element_iterator& rObjectElement, Object *aObject, int versionNumber);
	static void InitializeObjectFromXMLNode2(AbstractProperty *aAbstractProperty, const SimTK::Xml::element_iterator& rObjectElement, Object *aObject, int versionNumber);
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
    // Each object type only appears once in this array. Renamed types do not 
    // have separate registered objects; they are just used to locate one of 
    // the current ones.
	static ArrayPtrs<Object>                    _registeredTypes;

	// Map from concrete object class name string to a default object of that 
    // type kept in the above array of registered types. Renamed types are *not* 
    // entered here; the names are mapped separately using the map below.
	static std::map<std::string,Object*>        _mapTypesToDefaultObjects;

	// Map types that have been renamed to their new names, which can
    // then be used to find them in the default object map. This lets us 
    // recognize the old names while converting to the new ones internally
    // so that they will be updated when written out.
	static std::map<std::string,std::string>    _renamedTypesMap;

	// Global flag to indicate if all registered objects are to be written in 
    // a "defaults" section.
	static bool _serializeAllDefaults;

	// Debug level: 
	//	0: Hides non fatal warnings 
	//  1: Shows illegal tags 
	//  2: level 1 + registration troubleshooting
	//  3: 2 + more verbose troubleshooting of Object (de)serialization. When 
    //     used from ava wrapping in GUI/Matlab this catches all exceptions 
    //     thrown by the low level libraries which is slower but helpful in 
    //     troubleshooting.
	static int      _debugLevel;

	// Property table for serializable properties of this and derived classes.
	PropertyTable   _propertyTable;

	// The name of this object.
	std::string     _name;
	// A short description of the object.
	std::string     _description;

	// List of authors who contributed to the implementation of concrete object.
	std::string     _authors;
	// List of references that should be cited when using this concrete object.
	std::string     _references;

	// The XML document, if any, associated with this object.
	XMLDocument     *_document;
	// Flag indicating whether the object is serialized to this _document or 
    // to another fresh document.
	bool            _inlined;

//==============================================================================
};	// END of class Object



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
getProperty(const std::string& name) const {
    // Look it up by T's object class name if no name is given.
    if (Property<T>::TypeHelper::IsObjectType && name.empty()) {
        return _propertyTable.getProperty<T>
            (Property<T>::TypeHelper::getTypeName());
    }
	return _propertyTable.getProperty<T>(name);
}

template <class T> Property<T>& Object::
updProperty(const std::string& name) {
    // Look it up by T's object class name if no name is given.
    if (Property<T>::TypeHelper::IsObjectType && name.empty()) {
        return _propertyTable.updProperty<T>
            (Property<T>::TypeHelper::getTypeName());
    }
	return _propertyTable.updProperty<T>(name);
}

template <class T> Property<T>& Object::
addProperty(const std::string& name, const std::string& comment, 
            const T& value)
{
    // Restrict to exactly one value. If there is no name, this will throw
    // an exception if T is a simple (non-object) type.
    Property<T>* p = Property<T>::TypeHelper::create(name, true);

    p->setComment(comment);
    p->appendValue(value);

    // Note that an unnamed, one-object property will use the object class name
    // as a name for lookup purposes.
	_propertyTable.adoptProperty(p);
    return *p;
}

template <class T> Property<T>& Object::
addOptionalProperty(const std::string& name, const std::string& comment, 
                    const T& value)
{
    if (name.empty())
        throw OpenSim::Exception(
            "Object::addOptionalProperty(): an optional property must have "
            "a name. (Object " + getName() + ").");

    Property<T>* p = Property<T>::TypeHelper::create(name, false);
    p->setAllowableListSize(0,1);
    p->setComment(comment);
    p->appendValue(value);

	_propertyTable.adoptProperty(p);
    return *p;
}

template <class T> Property<T>& Object::
addOptionalProperty(const std::string& name, const std::string& comment)
{
    if (name.empty())
        throw OpenSim::Exception(
            "Object::addOptionalProperty(): an optional property must have "
            "a name. (Object " + getName() + ").");

    Property<T>* p = Property<T>::TypeHelper::create(name, false);
    p->setAllowableListSize(0,1);
    p->setComment(comment);

	_propertyTable.adoptProperty(p);
    return *p;
}

template <class T> Property<T>& Object::
addListProperty(const std::string& name, 
                const std::string& comment)
{
    if (name.empty())
        throw OpenSim::Exception(
            "Object::addListProperty(): a list property must have a name. "
            "(Object " + getName() + ").");

    // No array size restrictions.
    Property<T>* p = Property<T>::TypeHelper::create(name, false);
    p->setName(name); p->setComment(comment);

	_propertyTable.adoptProperty(p);
    return *p;
}

template <class T, template<class> class Container> Property<T>& Object::
addListProperty(const std::string&  name, 
                const std::string&  comment,
                const Container<T>& valueList)
{
    if (name.empty())
        throw OpenSim::Exception(
            "Object::addListProperty(): a list property must have a name. "
            "(Object " + getName() + ").");

    // No array size restrictions.
    Property<T>* p = Property<T>::TypeHelper::create(name, false);
    p->setName(name); p->setComment(comment);
    for (int i=0; i < (int)valueList.size(); ++i)
        p->appendValue(valueList[i]);

	_propertyTable.adoptProperty(p);
    return *p;
}


template <class T, template<class> class Container> void Object::
setPropertyValue(const std::string& name, 
                 const Container<T>& valueList)
{
    const int sz = (int)valueList.size();
    Property<T>& dest = updProperty<T>(name);
    if (sz < dest.getMinListSize() || sz > dest.getMaxListSize())
        throw OpenSim::Exception(
            "Object::setListPropertyValue(): supplied list length "
            + SimTK::String(sz) + " out of range for property " + dest.getName());
    dest.clear();
    for (int i=0; i < sz; ++i)
        dest.appendValue(valueList[i]);
}

template <class T> void Object::
setPropertyValue(const std::string& name, 
                 const Property<T>& source)
{
    const int sz = source.size();
    Property<T>& dest = updProperty<T>(name);
    if (sz < dest.getMinListSize() || sz > dest.getMaxListSize())
        throw OpenSim::Exception(
            "Object::setListPropertyValue(): number of values ("
            + SimTK::String(sz) + ") in source property " + source.getName()
            + " out of range for property " + dest.getName());
    dest.clear();

    for (int i=0; i < sz; ++i)
        dest.appendValue(source.getValue(i));

    dest.setUseDefault(source.getUseDefault());
}

template <class T> const T& Object::
getPropertyValue(const std::string& name) const
{
	return getProperty<T>(name).getValue();
}

template <class T> T& Object::
updPropertyValue(const std::string& name)
{
	return updProperty<T>(name).updValue();
}

template <class T> void Object::
setPropertyValue(const std::string& name, const T& value)
{
    Property<T>& prop = updProperty<T>(name);
    if (prop.empty())
        prop.appendValue(value);
    else 
	    prop.updValue() = value;
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
/** @cond **/ 

// This class allows us to get the class name for template arguments using
// getClassName() when it is available, otherwise a specialization.
template <class T> struct Object_GetClassName 
{   static const std::string& name() {return T::getClassName();} };
template <> struct Object_GetClassName<double> 
{   static const std::string name() {return "double";} };
template <> struct Object_GetClassName<int> 
{   static const std::string name() {return "int";} };
template <> struct Object_GetClassName<bool> 
{   static const std::string name() {return "bool";} };
template <> struct Object_GetClassName<std::string> 
{   static const std::string name() {return "string";} };
template <> struct Object_GetClassName<SimTK::Vec3> 
{   static const std::string name() {return "Vec3";} };

#define OpenSim_OBJECT_ANY_DEFS(ConcreteClass, SuperClass)                     \
public:                                                                        \
typedef SuperClass Super;                                                      \
OpenSim_OBJECT_JAVA_DEFS(ConcreteClass, SuperClass);

// For nontemplate classes, the class name is identical to the supplied
// ConcreteClass argument.
#define OpenSim_OBJECT_NONTEMPLATE_DEFS(ConcreteClass, SuperClass)             \
static std::string& getClassName()                                             \
{   static std::string name(#ConcreteClass); return name; }

// For template classes ConcreteClass<TemplateArg>, we construct the class
// name by assembling the pieces.
#define OpenSim_OBJECT_TEMPLATE_DEFS(ConcreteClass, TArg, SuperClass)          \
static std::string& getClassName()                                             \
{   static std::string name = #ConcreteClass "<"                               \
                              + Object_GetClassName<TArg>::name()              \
                              + ">";                                           \
    return name; }

// This provides definitions for the two Object pure virtuals clone() and
// getConcreteClassName().
#define OpenSim_OBJECT_CONCRETE_DEFS(ConcreteClass)                            \
ConcreteClass* clone() const OVERRIDE_11 {return new ConcreteClass(*this);}    \
const std::string& getConcreteClassName() const OVERRIDE_11                    \
{   return getClassName(); }                                                   \
private:

// This leaves the two Object pure virtuals clone() and getConcreteClassName()
// unimplemented, but changes the return type of clone() to ConcreteClass*,
// which allows it to be invoked ConcreteClass::clone() and return the correct
// pointer type.
#define OpenSim_OBJECT_ABSTRACT_DEFS(ConcreteClass)                            \
ConcreteClass* clone() const OVERRIDE_11 = 0;                                  \
const std::string& getConcreteClassName() const OVERRIDE_11 = 0;               \
private:

// Add public static method declaration in class derived from a
// parent to assist in downcasting objects of the parent type to the 
// derived type as well as support dynamic casting across JNI.
// TODO: (sherm 20120415) I'm not sure if any or all of these methods are
// still needed -- Ayman?
#define OpenSim_OBJECT_JAVA_DEFS(thisClass,parentclass) \
	typedef parentclass Parent; \
  public: \
  static bool isKindOf(const char *type) \
  { \
    if (strcmp(#thisClass,type)==0) \
      { \
      return true; \
      } \
    return Parent::isKindOf(type); \
  } \
  virtual bool isA(const char *type) const \
  { \
    return this->thisClass::isKindOf(type); \
  } \
  static thisClass* safeDownCast(OpenSim::Object *obj) \
  { \
      return dynamic_cast<thisClass *>(obj); \
  } \
  virtual void copy(const Object &aObject) \
  { \
	  if (aObject.isA(#thisClass)) { \
		  *this = *((const thisClass*)(&aObject)); \
     } else { \
	  throw OpenSim::Exception(std::string(#thisClass)+ \
              "::copy() called with object (name = " + aObject.getName() \
              + ", type = " + aObject.getConcreteClassName()+").", \
              __FILE__,__LINE__); \
     } \
  }
/** @endcond **/

//==============================================================================
//                        OBJECT PROPERTY IMPLEMENTATION
//==============================================================================
// These methods of ObjectProperty are defined here because they depend on 
// methods of Object. See Property.h for ObjectProperty's declaration.

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
    if (this->getUseDefault() != other.getUseDefault())
        return false;
    assert(this->size() == other.size()); // base class checked
    const ObjectProperty& otherO = ObjectProperty::getAs(other);
    for (int i=0; i<objects.size(); ++i)
        if (!(objects[i] == otherO.objects[i]))
            return false;
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
template <class T> inline void 
ObjectProperty<T>::writeToXMLElement
    (SimTK::Xml::Element& propertyElement) const 
{
    for (int i=0; i < objects.size(); ++i)
        const_cast<T&>(*objects[i]).updateXMLNode(propertyElement);
}


template <class T> inline void 
ObjectProperty<T>::setValueAsObject(const Object& obj, int index) {
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
        return pd->isArrayProperty()
            ? pd->getValueArray<T>()[index]
            : pd->getValue<T>();
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
